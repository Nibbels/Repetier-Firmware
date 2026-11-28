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
    if not matches:
        return None  # Changed or absent landmark requires manual review.
    if len(matches) != 1:
        raise ValueError(f"linked landmark needs review: {pattern} ({len(matches)} matches)")
    return matches[0]


def generate(archive, archive_hash, merge, allow_failed_replays=False):
    if digest(archive.read_bytes()) != archive_hash:
        raise ValueError("archive digest mismatch")
    boards = {}
    with zipfile.ZipFile(archive) as z:
        summary = json.loads(z.read("summary.json"))
        if summary["sha"] != merge or (summary["failures"] and not allow_failed_replays) or set(summary["targets"]) != set(BOARDS):
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
    return {"archiveFailures":summary["failures"], "merge":merge, "archiveSha256":archive_hash, "boards":boards}


def linked_analysis(archive, report, reviewed_owners):
    import runpy
    from lease_gpio_latency import gpio_latency
    directory = Path(__file__).resolve().parent
    context = runpy.run_path(str(directory / "contextual-stack-diagnostic.py"))
    irq = runpy.run_path(str(directory / "irq-mask-cfg.py"))
    startup = runpy.run_path(str(directory / "startup-stack.py"))
    switches = runpy.run_path(str(directory / "switch-table-guards.py"))
    foreground_masks = runpy.run_path(str(directory / "foreground-irq-masks.py"))
    foreground_masks["self_test"](irq)
    fg = context["fg"]
    with zipfile.ZipFile(archive) as z:
        for board, row in report["boards"].items():
            base = board + "/"
            listing = z.read(base + "disassembly.txt").decode()
            symbols = z.read(base + "symbols.txt").decode()
            image = startup["elf_text"](z.read(base + "build/Repetier.ino.elf"))
            row["crt"] = startup["verify"](board, image, listing, symbols, fg)
            instructions, tables, labels = fg.decode_linked(listing, board)
            row["leaseGpio"] = gpio_latency(board, instructions, labels, fg.target)
            validated = switches["prove_tables"](instructions, tables, labels, fg.target)
            row["verifiedSwitchTables"] = len(validated["tables"])
            row["foreground"] = context["inventory"](z, board, False, reviewed_owners)
            fingerprint = context["owner_fingerprint"](z, base)
            row["sourceOwnershipFingerprint"] = fingerprint
            if fingerprint in context["REVIEWED_SOURCE_SHA256"]:
                row["reviewedForeground"] = (row["foreground"] if reviewed_owners else
                                             context["diagnostic"](fg.inventory(z, board, True), False, True))
            else:
                row["reviewedForeground"] = None  # Requires new source review; never silently prune.
            row["interrupts"] = irq["inventory"](z, board, True)
            row["foregroundMask"] = foreground_masks["inventory"](z, board, fg, irq)
            timer = row["interrupts"][17]
            envelope = timer["stackBytesIncludingPc"] + max(
                item["stackBytesIncludingPc"] for v,item in row["interrupts"].items() if v != 17)
            row["conditionalInterruptBytes"] = envelope
            row["conditionalFreeBytes"] = row["staticFree"] - envelope - row["foreground"]["diagnosticBytes"]
            row["reviewedConditionalFreeBytes"] = (None if row["reviewedForeground"] is None else
                row["staticFree"] - envelope - row["reviewedForeground"]["diagnosticBytes"])
    report["reviewedOwners"] = reviewed_owners
    report["releaseProof"] = False


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("archive", type=Path)
    p.add_argument("archive_sha256")
    p.add_argument("tested_merge")
    p.add_argument("--allow-failed-replays", action="store_true", help="identify verified images for diagnosis and retain every archive failure")
    p.add_argument("--linked", action="store_true", help="run startup, switch, foreground and ISR analyses")
    p.add_argument("--reviewed-owners", action="store_true", help="apply source-pinned foreground ownership exclusions")
    args = p.parse_args()
    if args.reviewed_owners and not args.linked:
        p.error("--reviewed-owners requires --linked")
    report = generate(args.archive, args.archive_sha256, args.tested_merge, args.allow_failed_replays)
    if args.linked:
        linked_analysis(args.archive, report, args.reviewed_owners)
    print(json.dumps(report, indent=2))
