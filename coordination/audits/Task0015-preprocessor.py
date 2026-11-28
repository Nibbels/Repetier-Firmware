#!/usr/bin/env python3
"""Task0015: compare actual compiler macro use and include graph, four variants."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

TARGETS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
REMOVED = {"UI_PRINTER_COMPANY", "VIRTUAL_AXIS"}
NEW_GUARDS = {"BOARD_CONFIGURATION_H", "R_MATH_H"}
DEFINE = re.compile(r"^#define ([A-Za-z_]\w*)(.*)$", re.M)
TRACE = re.compile(r"^#(?:define|undef) ([A-Za-z_]\w*)\b", re.M)
ROOT = Path(__file__).resolve().parents[2]


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT)


def flags(entry):
    args = entry["arguments"]
    n = args.index("-o")
    return [a for a in args[:n] + args[n+2:] if a not in ("-c", "-MMD")]


def preprocess(entry, args, *opts):
    return subprocess.run([*args, "-E", *opts], cwd=entry["directory"],
                          text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)


def inspect_variant(evidence, model, base_dir):
    build = evidence / model / "build"
    sketch = build / "sketch"
    db = json.loads((build / "compile_commands.json").read_text())
    units = [entry for entry in db if Path(entry["file"]).is_relative_to(sketch)]
    expected = {p.name for p in (ROOT / "Repetier").glob("*.cpp")}
    expected.add("Repetier.ino.cpp")
    assert {Path(entry["file"]).name for entry in units} == expected
    baseline = base_dir / model
    baseline.mkdir()
    for p in (base_dir / "Repetier").iterdir():
        if p.suffix in (".h", ".cpp"):
            (baseline / p.name).write_bytes(p.read_bytes())
    (baseline / "Repetier.ino.cpp").write_bytes((sketch / "Repetier.ino.cpp").read_bytes())
    result = {"units": {}, "baseline_observed_removed": [], "candidate_observed_removed": []}
    old_seen, new_seen = set(), set()
    for entry in units:
        unit = Path(entry["file"]).name
        args = flags(entry)
        assert Path(args[0]).name == "avr-g++"
        old_args = [a.replace(str(sketch), str(baseline)).replace(str(ROOT / "Repetier"), str(baseline))
                    for a in args]
        current = preprocess(entry, args, "-dU", "-H")
        old = preprocess(entry, old_args, "-dU")
        old_seen.update(TRACE.findall(old.stdout))
        new_seen.update(TRACE.findall(current.stdout))
        result["units"][unit] = [line for line in current.stderr.splitlines()
                                  if re.match(r"^\.+ ", line)]
        if unit == "Repetier.ino.cpp":
            before = dict(DEFINE.findall(preprocess(entry, old_args, "-dM").stdout))
            after = dict(DEFINE.findall(preprocess(entry, args, "-dM").stdout))
            assert REMOVED <= before.keys() and not REMOVED & after.keys()
            delta = {k: [before.get(k), after.get(k)] for k in before.keys() | after.keys()
                     if before.get(k) != after.get(k)}
            result["umbrella_macro_delta"] = delta
            assert set(delta) == REMOVED | NEW_GUARDS, delta
        if unit == "HAL.cpp":
            before = dict(DEFINE.findall(preprocess(entry, old_args, "-dM").stdout))
            after = dict(DEFINE.findall(preprocess(entry, args, "-dM").stdout))
            keys = ("__AVR_ATmega2560__", "ADCSRB", "MUX5", "MOTHERBOARD", "WATCHDOG_PIN",
                    "NUM_EXTRUDER", "EXT0_EXTRUDER_COOLER_PIN", "I2C_ADDRESS_STRAIN_GAUGE",
                    "I2C_ADDRESS_EXTERNAL_EEPROM", "I2C_ADDRESS_TYPE_EEPROM")
            assert all(k in before and before[k] == after.get(k) for k in keys)
            result["board_contract"] = {k: after[k].strip() for k in keys}
    result["baseline_observed_removed"] = sorted(old_seen & REMOVED)
    result["candidate_observed_removed"] = sorted(new_seen & REMOVED)
    assert not (old_seen | new_seen) & REMOVED, result
    print(f"{model}: {len(units)} AVR units; removed macros not observed; board and mux definitions unchanged", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("base")
    args = parser.parse_args()
    evidence = args.evidence.resolve()
    summary = json.loads((evidence / "summary.json").read_text())
    assert not summary["failures"] and summary["sha"] == git("rev-parse", "HEAD").decode().strip()
    with TemporaryDirectory(prefix="rfx-task0015-") as d:
        directory = Path(d)
        # The baseline source is exactly the selected target, not a reconstruction of defines.
        archive = subprocess.Popen(["git", "archive", args.base, "Repetier"], cwd=ROOT, stdout=subprocess.PIPE)
        subprocess.run(["tar", "-x", "-C", str(directory)], stdin=archive.stdout, check=True)
        archive.stdout.close()
        assert archive.wait() == 0
        with ThreadPoolExecutor(max_workers=4) as pool:
            tasks = {model: pool.submit(inspect_variant, evidence, model, directory) for model in TARGETS}
            report = {model: future.result() for model, future in tasks.items()}
    (evidence / "task0015-preprocessor.json").write_text(json.dumps({
        "baseline": args.base, "sha": summary["sha"], "variants": report}, indent=2) + "\n")


if __name__ == "__main__":
    main()
