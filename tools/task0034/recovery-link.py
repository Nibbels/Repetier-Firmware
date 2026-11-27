"""Task0034: guarded experiment for the Mega2560 EIND watchdog boot bug.

Arduino's historic STK500v2 loader can EICALL word address 0x10000
(byte 0x20000) after disabling WDT. Reserve that address in the linked
application; never patch bytes over ordinary code after linking.
See HAL.cpp and coordination/tasks/Task0034.md A15/A16.
"""
import json
import re
import subprocess

RECOVERY = 0x20000
BOOT_START = 0x3e000
ENTRY_SIZE = 30


def make_script(bin_dir, directory, fill_named_text=False, partitions=(0,), probe=False):
    # Retain the pinned toolchain's own memory, vector, PROGMEM, trampoline,
    # CRT, .data LMA, .noinit, EEPROM and debug section rules. Fail closed
    # when the expected script shape/partition layout changes.
    raw = subprocess.check_output([str(bin_dir / "avr-ld"), "-m", "avr6", "--verbose"],
                                  text=True)
    match = re.search(r"(?ms)^=+\s*\n(.*?)^=+\s*$", raw)
    if not match:
        raise ValueError("Cannot extract the AVR6 default linker script")
    script = match[1]
    if not re.search(r"OUTPUT_ARCH\s*\(\s*avr:6\s*\)", script):
        raise ValueError("Wrong linker emulation")
    # The current pinned LTO emits large indivisible .text input blocks.
    # Place ltrans0 first, then the recovery entry, then all remaining code.
    # A future compiler may make the low block too large; a backwards
    # location-counter assignment intentionally fails rather than silently
    # moving the entry. Padding is explicitly a first experiment, not an
    # optimized final layout.
    old = "*(.text)"
    if script.count(old) != 1:
        raise ValueError("Unexpected .text wildcard in default script")
    snippet = """/* RFx000 historical STK500v2 watchdog/EIND compatibility.
       Byte 0x20000 is WORD PC 0x10000, not an ordinary callable function.
       Keep this even if no call sites exist; see HAL.cpp Task0034. */
    __rf_near_parser_start = ABSOLUTE(.);
    *(.text.rf_near_parser)
    __rf_near_parser_end = ABSOLUTE(.);
    ASSERT(__rf_near_parser_end > __rf_near_parser_start, "RF near parser missing");
    ASSERT(__rf_near_parser_end <= 0x20000, "RF near parser beyond word range");
    *ltrans0.ltrans.o(.text)
    __rf_watchdog_padding_start = ABSOLUTE(.);
    . = 0x20000;
    __rf_watchdog_recovery_link_guard = ABSOLUTE(.);
    KEEP(*(.rf_watchdog_recovery))
    ASSERT(heaterWatchdogRecoveryEntry == 0x20000, "RF recovery entry moved");
    ASSERT((. - __rf_watchdog_recovery_link_guard) == 30, "RF recovery entry size changed");
    *(.text)"""
    if fill_named_text:
        # Move the existing ordinary named code sections, not near
        # PROGMEM, vectors, CRT init/fini or the recovery entry.
        # The ltrans0 block remains last before padding, so the existing
        # no-fallthrough audit still applies. Never change entry address.
        if script.count("*(.text.*)") != 1:
            raise ValueError("Unexpected named-text wildcard")
        snippet = snippet.replace("*ltrans0.ltrans.o(.text)", "*(.text.*)\n    *ltrans0.ltrans.o(.text)")
    if tuple(sorted(set(partitions))) != tuple(partitions) or any(not isinstance(p,int) or p<0 or p>63 for p in partitions):
        raise ValueError("Invalid LTO partition selection")
    snippet = snippet.replace("*ltrans0.ltrans.o(.text)", "\n    ".join(f"*ltrans{p}.ltrans.o(.text)" for p in partitions))
    if probe:
        # Diagnostic link only: no fixed recovery address or padding.
        # Never pass this ELF to verify_image or publish it as a candidate.
        snippet = snippet.replace(". = 0x20000;", "/* sizing probe: no reserved hole */")
        snippet = snippet.replace('ASSERT(heaterWatchdogRecoveryEntry == 0x20000, "RF recovery entry moved");',
                                  '/* sizing probe: recovery address intentionally unconstrained */')
    script = script.replace(old, snippet)
    path = directory / "avr6-recovery.x"
    path.write_text(script)
    print("BEGIN GENERATED AVR6 RECOVERY SCRIPT")
    print(script)
    print("END GENERATED AVR6 RECOVERY SCRIPT", flush=True)
    return path


def named_text_retry(map_text, compile_log):
    """Only retry a size-rejected linked image with a proven padding opportunity.

    The first map is evidence for a candidate, not an assurance that LTO
    will reproduce sizes. A second link must pass all ordinary checks.
    """
    if "text section exceeds available space in board" not in compile_log:
        return None
    size = re.search(r"Sketch uses (\d+) bytes .*Maximum is (\d+) bytes", compile_log)
    padding = re.search(r"^\s*(0x[0-9a-f]+)\s+__rf_watchdog_padding_start = ABSOLUTE", map_text, re.M)
    if not size or not padding:
        return None
    used, maximum = map(int,size.groups())
    if maximum != BOOT_START or used <= maximum:
        return None
    gap = RECOVERY-int(padding[1],16)
    # The pinned default script places this wildcard immediately before
    # the align/fini sequence. Count only live ordinary named text there.
    start=map_text.find("\n *(.text.*)", map_text.find("__rf_watchdog_recovery_link_guard"))
    if start<0:
        return None
    # Parse the section rows (some names wrap onto a separate line).
    block=map_text[start:]
    stop=re.search(r"^\s*0x[0-9a-f]+\s+\. = ALIGN",block,re.M)
    if not stop:
        return None
    rows=re.findall(r"^\s+\.text\.\S+\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)\s+\S+",block[:stop.start()],re.M)
    named=sum(int(n,16) for n in rows)
    # Leave a conservative 256-byte layout margin on both sides;
    # if this cannot fit, keep the failing build and investigate.
    if named < used-maximum+256 or named+256 > gap:
        return None
    return {"used":used,"maximum":maximum,"padding":gap,"named_text":named,
            "projected_used":used-named}


class NoProbeLayout(ValueError):
    """A complete sizing map has no feasible whole-block layout."""


def select_probe_layout(map_text, used):
    """Choose whole input blocks from a successful no-hole sizing link.
    All ordinary final image audits still run; never assume LTO stability.
    """
    import itertools
    m = re.search(r"^\s*(0x[0-9a-f]+)\s+__rf_near_parser_end = ABSOLUTE", map_text, re.M)
    if not m:
        raise ValueError("No complete near parser end in probe map")
    start = int(m[1],16)
    if not 0 < start < RECOVERY or used > BOOT_START:
        raise ValueError("Invalid or oversized sizing probe")
    rows = re.findall(r"^\s+\.text\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)\s+\S*\.ltrans(\d+)\.ltrans\.o\s*$", map_text, re.M)
    blocks = {}
    for size, index in rows:
        index = int(index)
        if index in blocks:
            raise ValueError("Ambiguous LTO block")
        blocks[index] = int(size,16)
    if not 1 <= len(blocks) <= 16 or 0 not in blocks:
        raise ValueError("Unexpected LTO partition inventory")
    # Named sections after the recovery entry, excluding parser and CRT.
    guard = map_text.index("__rf_watchdog_recovery_link_guard")
    begin = map_text.index("\n *(.text.*)", guard)
    block = map_text[begin:]
    stop = re.search(r"^\s*0x[0-9a-f]+\s+\. = ALIGN",block,re.M)
    if not stop:
        raise ValueError("Missing end of named text")
    named = sum(int(n,16) for n in re.findall(
        r"^\s+\.text\.\S+\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)\s+\S+",block[:stop.start()],re.M))
    options=[]
    ids=sorted(blocks)
    for count in range(1,len(ids)+1):
        for chosen in itertools.combinations(ids,count):
            for fill in (False,True):
                if fill and not named:continue
                gap=RECOVERY-start-sum(blocks[i] for i in chosen)-(named if fill else 0)
                if gap>=256 and used+gap+256<=BOOT_START:
                    options.append((gap,fill,chosen))
    if not options:
        raise NoProbeLayout("No whole-partition layout with 256-byte margins")
    gap,fill,chosen=min(options)
    return {"partitions":list(chosen),"fill_named_text":fill,"padding":gap,
            "projected_used":used+gap,"probe_used":used,"parser_end":start,
            "blocks":blocks,"named_text":named}


def decode_hex(path):
    image = {}
    base = 0
    eof = False
    for line in path.read_text().splitlines():
        if not line.startswith(":") or eof:
            raise ValueError("Malformed Intel HEX")
        record = bytes.fromhex(line[1:])
        size, hi, lo, kind = record[:4]
        if len(record) != size + 5 or sum(record) & 255:
            raise ValueError("Bad Intel HEX checksum/length")
        addr = (hi << 8) | lo
        data = record[4:-1]
        if kind == 0:
            for i, byte in enumerate(data):
                pos = base + addr + i
                if pos in image:
                    raise ValueError("Overlapping HEX records")
                image[pos] = byte
        elif kind in (2, 4):
            if size != 2 or addr:
                raise ValueError("Malformed extended address")
            base = int.from_bytes(data, "big") << (4 if kind == 2 else 16)
        elif kind == 1:
            if size or addr:
                raise ValueError("Malformed EOF")
            eof = True
        elif kind not in (3, 5):
            raise ValueError("Unexpected Intel HEX record")
    if not eof or not image:
        raise ValueError("Incomplete Intel HEX")
    return image



def proven_parser_tables(instructions):
    """Recognize the pinned GCC7 bounded inline switch word table only.

    Linear objdump decodes table data as instructions (0x95a8 can be
    a code pointer, not a WDR). Do not whitelist a function or WDR address.
    Validate the dispatch instruction sequence, derived table base/size,
    actual table words and all destinations before excluding the data.
    """
    def word(ins): return int.from_bytes(ins[1],"little")
    def imm(w): return (w&15)|((w>>4)&0xf0)
    def target(ins): return int(re.search(r"(?:^|;\s*)0x([0-9a-f]+)\b",ins[3])[1],16)
    by_addr={ins[0]:ins for ins in instructions}
    flash={ins[0]+j:b for ins in instructions for j,b in enumerate(ins[1])}
    helper=[ins[1].hex() for ins in instructions if ins[4]=="__tablejump2__"]
    tables=[]
    for i,ins in enumerate(instructions):
        if ins[4]!="GCodeInput::parseAscii(GCode&, char*, bool)" or ins[2]!="jmp" or "<__tablejump2__>" not in ins[3]: continue
        seq=instructions[i-11:i+1]
        if len(seq)!=12 or any(x[4]!=ins[4] for x in seq):continue
        w=[word(x) for x in seq]
        if (w[0]&0xf0f0)!=0x50e0 or (w[1]&0xf0f0)!=0x30e0 or w[2]!=0xf008 or (w[3]&0xf000)!=0xc000:continue
        if w[4:8]!=[0x2e0e,0x0c00,0x0bff,0x2788]:continue
        if [x&0xf0f0 for x in w[8:11]]!=[0x50e0,0x40f0,0x4080]:continue
        assert helper==['ee0f','ff1f','881f','8bbf','0790','f691','e02d','1994'],helper
        assert all(a[0]+len(a[1])==b[0] for a,b in zip(seq,seq[1:]))
        assert target(seq[2])==seq[4][0]
        count=imm(w[1]);assert 0<count<128
        start=ins[0]+len(ins[1]);end=start+2*count
        neg_base=imm(w[8])|(imm(w[9])<<8)|(imm(w[10])<<16)
        assert ((-neg_base)&0xffffff)*2==start
        assert target(seq[3])>=end
        words=[2*(flash[a]|flash[a+1]<<8) for a in range(start,end,2)]
        assert all(a in by_addr and by_addr[a][4]==ins[4] and not start<=a<end for a in words),words
        assert all(not (x[0]<start<x[0]+len(x[1]) or x[0]<end<x[0]+len(x[1])) for x in instructions)
        tables.append((start,end))
    return tables


def verify_watchdog_instructions(dump, cookie_address, padding):
    """Static linked audit, not a model of reset, silicon or WCET.

    Match actual disassembled instruction lines; ignore mixed-in C++ source.
    Reject a new WDR site, timed-register sequence or recovery branch until
    explicitly reviewed. This guard is intentionally pinned to AVR GCC7.
    """
    instructions = []
    function = ""
    for line in dump.splitlines():
        symbol = re.match(r"^[0-9a-f]+ <(.+)>:$", line)
        if symbol:
            function = symbol[1]
        match = re.match(r"^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2} )+[0-9a-f]{2})\s+(\S+)(?:\s+(.*))?$", line)
        if match:
            addr, raw, op, operands = match.groups()
            instructions.append((int(addr,16), bytes.fromhex(raw), op, operands or "", function))
    assert instructions
    tables=proven_parser_tables(instructions)
    instructions=[ins for ins in instructions if not any(a<=ins[0]<b for a,b in tables)]
    # The retained cookie has to lie beyond BSS, in the actual .noinit.
    noinit = re.search(r"^\s*\d+\s+\.noinit\s+([0-9a-f]+)\s+([0-9a-f]+)\s+",dump,re.M)
    assert noinit, "No linked .noinit"
    size,start = (int(x,16) for x in noinit.groups())
    assert start <= 0x800000+cookie_address <= start+size-4
    feeds = [(i, ins) for i, ins in enumerate(instructions) if ins[1] == b"\xa8\x95"]
    assert len(feeds)==3, feeds
    def following(index,n):
        return [ins[1].hex() for ins in instructions[index:index+n]]
    early = next(i for i,ins in feeds if ins[4]=="heaterWatchdogEarlyInit")
    assert following(early,4)==["a895","88e1","80936000","10926000"]
    enable = next(i for i,ins in feeds if re.fullmatch(r"HAL::startWatchdog\(\)(?: \[clone \.part\.\d+\])?",ins[4]))
    assert following(enable,4)==["a895","80936000","0fbe","90936000"]
    # Actual register values immediately before the enable sequence.
    # r25=0x0e (WDE | WDP2 | WDP1), r24=0x18 (WDCE | WDE).
    assert following(enable-4,4)==["88e1","9ee0","0fb6","f894"], instructions[enable-4:enable]
    thermal = next(ins for _,ins in feeds if ins[4].startswith("tryThermalWatchdogFeed("))
    # No fall-through from the low code partition into linker padding.
    padding_start = RECOVERY-padding
    last = next(ins for ins in reversed(instructions) if ins[0]<padding_start)
    assert last[0]+len(last[1])==padding_start and last[2] in ("ret","reti","jmp","rjmp"), last
    # No ordinary direct transfer to the special entry or its interior.
    # This is not proof against corrupted PC or arbitrary indirect transfers.
    for addr,raw,op,operands,fn in instructions:
        if op in ("jmp","call","rjmp","rcall") or op.startswith("br"):
            target = re.search(r"(?:^|;\s*)0x([0-9a-f]+)\b",operands)
            if target:
                dest=int(target[1],16)
                assert not padding_start <= dest < RECOVERY+ENTRY_SIZE, (addr,op,operands,fn)
                assert not any(a<=dest<b for a,b in tables), (addr,op,operands,fn)
    result={"wdr_sites":[{"address":hex(ins[0]),"function":ins[4]} for _,ins in feeds],
            "noinit_range":[hex(start),hex(start+size)],"direct_recovery_transfers":0,"proven_parser_table_ranges":[[hex(a),hex(b)] for a,b in tables]}
    print("Recovery watchdog instruction audit",json.dumps(result),flush=True)
    return result

def verify_image(directory):
    symbols = (directory / "symbols.txt").read_text()
    def symbol(name):
        m = re.search(r"^([0-9a-f]+)\s+(?:[0-9a-f]+\s+)?\w\s+"+re.escape(name)+r"$",
                      symbols, re.M)
        if not m:
            raise ValueError("Missing linked symbol: "+name)
        return int(m[1], 16)
    assert symbol("__vectors") == 0
    assert symbol("heaterWatchdogRecoveryEntry") == RECOVERY
    assert symbol("__rf_watchdog_recovery_link_guard") == RECOVERY
    parser = re.search(r"^([0-9a-f]+) ([0-9a-f]+) [tT] GCodeInput::parseAscii\(GCode&, char\*, bool\)$", symbols, re.M)
    assert parser, "Missing exact linked parser function"
    parser_start, parser_size = (int(x,16) for x in parser.groups())
    assert symbol("__rf_near_parser_start") <= parser_start < parser_start+parser_size <= symbol("__rf_near_parser_end") <= RECOVERY
    padding = RECOVERY-symbol("__rf_watchdog_padding_start")
    assert 0 <= padding < RECOVERY
    cookie = symbol("heaterResetCookie")-0x800000
    assert 0x200 <= cookie <= 0x21fc
    # Independently encode the intended machine instructions, not an
    # objdump source-text search. CLI, four LDI/STS pairs, absolute JMP 0.
    expected = bytearray(b"\xf8\x94")
    for i, value in enumerate((0x52, 0x6c, 0x9a, 0xd3)):
        ldi = 0xe080 | ((value & 0xf0)<<4) | (value & 0x0f)
        expected.extend(ldi.to_bytes(2,"little"))
        expected.extend(b"\x80\x93")
        expected.extend((cookie+i).to_bytes(2,"little"))
    expected.extend(b"\x0c\x94\x00\x00")
    assert len(expected) == ENTRY_SIZE
    build = directory / "build"
    images = list(build.glob("*.ino.hex"))
    assert len(images) == 1, images
    image = decode_hex(images[0])
    assert min(image) == 0 and max(image) < BOOT_START, "Application overlaps bootloader"
    actual = bytes(image[RECOVERY+i] for i in range(ENTRY_SIZE))
    assert actual == expected, (actual.hex(), expected.hex())
    props = dict(line.split("=",1) for line in
                 (directory/"properties-with-linker.txt").read_text().splitlines() if "=" in line)
    bin_dir = props["compiler.path"]
    elf = next(build.glob("*.elf"))
    text_bin = directory / "recovery-text.bin"
    from pathlib import Path
    subprocess.run([str(Path(bin_dir)/"avr-objcopy"), "-O", "binary", "-j", ".text",
                    str(elf),str(text_bin)],check=True)
    assert text_bin.read_bytes()[RECOVERY:RECOVERY+ENTRY_SIZE] == expected
    report = {"entry_byte_address":hex(RECOVERY), "entry_bytes":actual.hex(),
              "padding_bytes":padding, "cookie_sram_address":hex(cookie),
              "last_application_byte":hex(max(image))}
    report["watchdog_audit"] = verify_watchdog_instructions((directory/"disassembly.txt").read_text(),cookie,padding)
    # Replay the real bootloader from the pinned Arduino core, not a
    # synthetic EICALL at a guessed address. The restricted interpreter
    # stops at application entry (or the non-WDRF branch), and does not
    # model peripherals, oscillator timing or RWW protection.
    boot_path = Path(props["runtime.platform.path"])/"bootloaders/stk500v2/stk500boot_v2_mega2560.hex"
    boot_image = decode_hex(boot_path)
    import hashlib
    report["bootloader_hex_sha256"] = hashlib.sha256(boot_path.read_bytes()).hexdigest()
    payload = {"boot":boot_image, "app":{RECOVERY+i:b for i,b in enumerate(actual)}, "cookie":cookie}
    result = subprocess.check_output(["node",str(Path(__file__).with_name("boot-recovery-replay.cjs"))],
                                     input=json.dumps(payload),text=True)
    replay = json.loads(result)
    assert replay["cases"] == 36
    report["boot_replay"] = {"cases":replay["cases"], "wdt_entry":next(r for r in replay["results"] if r["cause"]==8)}
    print("Recovery boot opcode replay",result,flush=True)
    print("Recovery ELF/HEX verified",json.dumps(report),flush=True)
    return report
