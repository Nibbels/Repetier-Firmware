#!/usr/bin/env python3
"""Capture RFx000 host replays and four linked AVR resource inventories."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CORE = "1.8.6"
FQBN = "arduino:avr:mega:cpu=atmega2560"
TARGETS = {
    "RF1000": ["DEVICE_TYPE_RF1000"],
    "RF2000": ["DEVICE_TYPE_RF2000"],
    "RF2000v2": ["DEVICE_TYPE_RF2000v2"],
    "RF2000-X44": ["DEVICE_TYPE_RF2000", "NUM_EXTRUDER=1", "EXT0_EXTRUDER_COOLER_PIN=X44_PIN"],
}


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def run(directory, name, command):
    command = [str(arg) for arg in command]
    with (directory / "commands.txt").open("a", encoding="utf-8") as log:
        log.write(shlex.join(command) + "\n")
    path = directory / (name + ".txt")
    print(f"{name}: {shlex.join(command)}", flush=True)
    with path.open("w", encoding="utf-8") as output:
        subprocess.run(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, check=True)
    return path.read_text(encoding="utf-8", errors="replace")


def section_sizes(text):
    sections = {m[1]: int(m[2]) for m in re.finditer(r"^\s*(\.\S+)\s+(\d+)\s+\d+\s*$", text, re.M)}
    if not {".text", ".data", ".bss"} <= sections.keys():
        raise ValueError("Unexpected avr-size -A output; refusing to guess resource totals")
    flash = sections[".text"] + sections[".data"]
    sram = sections[".data"] + sections[".bss"] + sections.get(".noinit", 0)
    return {"sections": sections, "flash_bytes": flash, "global_sram_bytes": sram,
            "sram_remaining_before_stack_bytes": 8192 - sram}


def host_tests(out):
    host = out / "host"
    host.mkdir()
    cxx = os.environ.get("CXX", "c++")
    failures = []
    jobs = [(path.stem, [sys.executable, path]) for path in sorted((ROOT / "tests").glob("*.py"))]
    for stem in ("machine-inactivity", "z-offset"):
        exe = host / (stem + ".exe")
        jobs += [(stem + "-compile", [cxx, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                                      ROOT / "tests" / (stem + ".cpp"), "-o", exe]),
                 (stem, [exe])]
    for name, command in [("compiler-version", [cxx, "--version"]), *jobs]:
        try:
            run(host, name, command)
        except (OSError, subprocess.CalledProcessError) as error:
            failures.append(f"host/{name}: {error}")
    return failures


def build_target(out, cli, name, definitions):
    directory = out / name
    directory.mkdir()
    build = directory / "build"
    map_file = build / "firmware.map"
    flags = "-DMOTHERBOARD=" + definitions[0] + "".join(" -D" + d for d in definitions[1:])
    map_flags = f'-Xlinker -Map -Xlinker "{map_file.as_posix()}" -Wl,--cref'
    command = [cli, "compile", "--fqbn", FQBN, "--build-path", build,
               "--build-property", "compiler.cpp.extra_flags=" + flags,
               "--build-property", "compiler.c.elf.extra_flags=" + map_flags, ROOT / "Repetier"]
    properties = run(directory, "properties", [*command, "--show-properties=expanded"])
    props = dict(line.split("=", 1) for line in properties.splitlines() if "=" in line)
    if "compiler.path" not in props:
        raise ValueError("No expanded compiler.path in Arduino build properties")
    bin_dir = Path(props["compiler.path"])
    for tool in ("avr-gcc", "avr-g++", "avr-size", "avr-nm", "avr-objdump"):
        executable = bin_dir / (tool + (".exe" if os.name == "nt" else ""))
        run(directory, tool + "-version", [executable, "--version"])
    run(directory, "compile", [*command, "--verbose"])
    elf_files = list(build.glob("*.elf"))
    if len(elf_files) != 1 or not map_file.is_file() or not map_file.stat().st_size:
        raise ValueError("Expected exactly one linked ELF and a nonempty linker map")
    elf = elf_files[0]
    def tool(name):
        return bin_dir / (name + (".exe" if os.name == "nt" else ""))
    sizes = section_sizes(run(directory, "sections", [tool("avr-size"), "-A", elf]))
    run(directory, "size", [tool("avr-size"), elf])
    run(directory, "symbols", [tool("avr-nm"), "-a", "-S", "-C", elf])
    symbols = run(directory, "symbols-by-size", [tool("avr-nm"), "-S", "--size-sort", "--radix=d", "-C", elf])
    rows = []
    for line in symbols.splitlines():
        match = re.match(r"^\s*(\d+)\s+(\d+)\s+([bBdD])\s+(.+)$", line)
        if match:
            address, size = int(match[1]), int(match[2])
            if 0x800200 <= address < 0x802200:
                rows.append({"name": match[4], "bytes": size, "address": address})
    sizes["largest_sram_symbols"] = sorted(rows, key=lambda row: -row["bytes"])[:20]
    sizes["elf_sha256"] = hashlib.sha256(elf.read_bytes()).hexdigest()
    sizes["map_sha256"] = hashlib.sha256(map_file.read_bytes()).hexdigest()
    run(directory, "disassembly", [tool("avr-objdump"), "-h", "-d", "-S", "-C", "-z", elf])
    dependencies = sorted(str(path.relative_to(directory)) for path in build.rglob("*.d"))
    (directory / "dependency-files.txt").write_text("\n".join(dependencies) + "\n", encoding="utf-8")
    # Record deployable image identities in the CI log and summary as well as the archive.
    images = sorted(path for path in build.iterdir()
                    if path.is_file() and path.suffix in {".hex", ".bin", ".eep"})
    sizes["image_sha256"] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in images}
    return sizes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new evidence directory outside checkout")
    parser.add_argument("--cli", default="arduino-cli")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--host-only", action="store_true")
    modes.add_argument("--avr-only", action="store_true")
    args = parser.parse_args()
    out = args.output.resolve()
    if out == ROOT or ROOT in out.parents or out.exists():
        parser.error("output must be a new directory outside the checkout")
    if git("status", "--porcelain", "--untracked-files=all"):
        parser.error("checkout must be clean so all evidence refers to the recorded SHA")
    out.mkdir(parents=True)
    report = {"sha": git("rev-parse", "HEAD"), "arduino_core_required": CORE,
              "fqbn": FQBN, "targets": {}, "failures": [],
              "host_requested": not args.avr_only, "avr_requested": not args.host_only}
    if not args.avr_only:
        report["failures"] += host_tests(out)
    if not args.host_only:
        try:
            run(out, "arduino-cli-version", [args.cli, "version"])
            cores = run(out, "arduino-cores", [args.cli, "core", "list"])
            if not re.search(r"^arduino:avr\s+" + re.escape(CORE) + r"\s", cores, re.M):
                raise ValueError(f"Install the pinned core: arduino-cli core install arduino:avr@{CORE}")
            run(out, "arduino-libraries", [args.cli, "lib", "list"])
        except (OSError, ValueError, subprocess.CalledProcessError) as error:
            report["failures"].append(f"AVR prerequisites: {error}")
        else:
            for name, definitions in TARGETS.items():
                try:
                    report["targets"][name] = build_target(out, args.cli, name, definitions)
                except (OSError, ValueError, subprocess.CalledProcessError) as error:
                    report["failures"].append(f"{name}: {error}")
    if git("status", "--porcelain", "--untracked-files=all") or git("rev-parse", "HEAD") != report["sha"]:
        report["failures"].append("Checkout changed during capture; evidence is not reproducible")
    (out / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return bool(report["failures"])


if __name__ == "__main__":
    sys.exit(main())
