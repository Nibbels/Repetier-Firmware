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


def make_script(bin_dir, directory):
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
    *ltrans0.ltrans.o(.text)
    __rf_watchdog_padding_start = ABSOLUTE(.);
    . = 0x20000;
    __rf_watchdog_recovery_link_guard = ABSOLUTE(.);
    KEEP(*(.rf_watchdog_recovery))
    ASSERT(heaterWatchdogRecoveryEntry == 0x20000, "RF recovery entry moved");
    ASSERT((. - __rf_watchdog_recovery_link_guard) == 30, "RF recovery entry size changed");
    *(.text)"""
    script = script.replace(old, snippet)
    path = directory / "avr6-recovery.x"
    path.write_text(script)
    print("BEGIN GENERATED AVR6 RECOVERY SCRIPT")
    print(script)
    print("END GENERATED AVR6 RECOVERY SCRIPT", flush=True)
    return path


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
    enable = next(i for i,ins in feeds if ins[4]=="HAL::startWatchdog()")
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
                assert not padding_start <= int(target[1],16) < RECOVERY+ENTRY_SIZE, (addr,op,operands,fn)
    result={"wdr_sites":[{"address":hex(ins[0]),"function":ins[4]} for _,ins in feeds],
            "noinit_range":[hex(start),hex(start+size)],"direct_recovery_transfers":0}
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
    print("Recovery ELF/HEX verified",json.dumps(report),flush=True)
    return report
