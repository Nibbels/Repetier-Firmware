"""Bind A73 results to the four official linked images and their review addresses."""
import argparse
import hashlib
import json
import re
import zipfile
from pathlib import Path

BOARDS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def landmark(body, pattern):
    matches = re.findall(pattern, body, re.MULTILINE)
    if len(matches) != 1:
        raise ValueError(f"linked landmark needs review: {pattern} ({len(matches)} matches)")
    return matches[0]


def generate(archive, archive_hash, merge):
    if digest(archive.read_bytes()) != archive_hash:
        raise ValueError("archive digest mismatch")
    boards = {}
    with zipfile.ZipFile(archive) as z:
        summary = json.loads(z.read("summary.json"))
        if summary["sha"] != merge or summary["failures"] or set(summary["targets"]) != set(BOARDS):
            raise ValueError("unexpected merge, failed suite or incomplete board set")
        for board in BOARDS:
            target = summary["targets"][board]
            def read(name):
                return z.read(board + "/" + name)
            if json.loads(read("verification.json")) != {"status":"verified","recovery":True}:
                raise ValueError(f"{board}: unverified recovery binary")
            files = {"build/Repetier.ino.elf":target["elf_sha256"], "build/firmware.map":target["map_sha256"]}
            files.update({"build/" + name: value for name,value in target["image_sha256"].items()})
            for name, value in files.items():
                if digest(read(name)) != value:
                    raise ValueError(f"{board}: digest mismatch for {name}")
            asm = read("disassembly.txt")
            files["disassembly.txt"] = digest(asm)
            listing = asm.decode()
            # Source annotations only locate candidates for manual review.
            # The returned addresses alone establish neither admission nor I.
            derive = landmark(listing, r"if \(updatederived\)\s*\n[^\n]*\n\s*Printer::updateDerivedParameter\(\);\s*\n\s*([0-9a-f]+):[^\n]*\bcall\b[^\n]*<Printer::updateDerivedParameter\(\)>")
            poll = landmark(listing, r"while \(next == tx_\.tail\) \{\s*\n\s*([0-9a-f]+):[^\n]*\blds\b[^\n]*<UsbSerial::tx_\+0x41>")
            boards[board] = {"hashes":files,"staticFree":target["sram_remaining_before_stack_bytes"],
                             "microstepDerivedCall":derive,"txTailPoll":poll}
    return {"merge":merge, "archiveSha256":archive_hash, "boards":boards}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("archive", type=Path)
    p.add_argument("archive_sha256")
    p.add_argument("tested_merge")
    args = p.parse_args()
    print(json.dumps(generate(args.archive, args.archive_sha256, args.tested_merge), indent=2))
