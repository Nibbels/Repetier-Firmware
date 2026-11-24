#!/usr/bin/env python3
"""Inventory compiler-observed source, guards and static storage for four AVR builds."""
import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import runpy
import shlex
import subprocess
from tempfile import TemporaryDirectory


TARGETS = ("RF1000", "RF2000", "RF2000v2", "RF2000-X44")
LINE = re.compile(r'^# (\d+) "([^"]+)"(?: [\d ]+)?$')
CONDITION = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b")
MACRO = re.compile(r"^\s*#\s*(define|undef)\s+([A-Za-z_]\w*)\b", re.M)
SYMBOL = re.compile(r"^.+/(\d+) \((.*)\) @0x[0-9a-f]+$", re.M)
ACCESS = re.compile(r"/(\d+) \((read|write|addr)\)")


def source_lines(output, sketch, source):
    active = defaultdict(set)
    positions = {}
    path = None
    number = 0
    for line in output.splitlines():
        marker = LINE.fullmatch(line)
        if marker:
            number = int(marker[1])
            filename = marker[2]
            if filename not in positions:
                positions[filename] = None
                for directory in (sketch, source):
                    try:
                        positions[filename] = Path(filename).relative_to(directory).as_posix()
                        break
                    except ValueError:
                        pass
            path = positions[filename]
        else:
            if path and line.strip():
                active[path].add(number)
            number += 1
    return active


def branch_spans(rows):
    stack = []
    for number, row in enumerate(rows, 1):
        match = CONDITION.match(row)
        if match is None:
            continue
        if match[1] in ("if", "ifdef", "ifndef"):
            stack.append(number)
        else:
            yield stack.pop(), number
            if match[1] in ("elif", "else"):
                stack.append(number)


def cgraph_states(dump, unit, states):
    prefix = "Initial Symbol table:\n"
    if prefix not in dump:
        raise ValueError(f"No initial GCC symbol table for {unit}")
    table = dump.split(prefix, 1)[1]
    table = table.split("\nRemoving unused symbols:", 1)[0]
    symbols = list(SYMBOL.finditer(table))
    names = {item[1]: item[2] for item in symbols}
    for item, following in zip(symbols, [*symbols[1:], None]):
        lines = table[item.end():following.start() if following else len(table)].splitlines()
        fields = dict(line.lstrip().split(": ", 1) for line in lines if ": " in line)
        kind = fields.get("Type", "").split()
        if not kind or kind[0] != "variable" or "read-only" in fields.get("Varpool flags", "").split():
            continue
        public = "public" in fields.get("Visibility", "").split()
        owner = "cross-TU" if public else f"{unit}/{item[1]}"
        state = states.setdefault((item[2], owner), defaultdict(set))
        if "definition" in kind:
            state["defined_in"].add(unit)
        for caller, access in ACCESS.findall(fields.get("Referring", "")):
            state[access].add(f"{unit}: {names[caller]}")


def inventory_target(build, source):
    sketch = build / "sketch"
    database = json.loads((build / "compile_commands.json").read_text())
    project_units = [entry for entry in database if Path(entry["file"]).is_relative_to(sketch)]
    expected_units = {p.relative_to(source).as_posix() for p in source.rglob("*.cpp")}
    expected_units.update(f"{p.relative_to(source).as_posix()}.cpp" for p in source.rglob("*.ino"))
    units = {Path(entry["file"]).relative_to(sketch).as_posix() for entry in project_units}
    if units != expected_units or len(units) != len(project_units):
        raise ValueError(f"Incomplete compilation database for {build}")
    for path in source.rglob("*"):
        if path.suffix in (".cpp", ".h"):
            copied = sketch / path.relative_to(source)
            original = path.read_bytes()
            line_marker = f'#line 1 "{path.as_posix()}"\n'.encode()
            if copied.read_bytes().removeprefix(line_marker) != original:
                raise ValueError(f"Sketch copy differs from source: {path}")

    included = defaultdict(set)
    active = defaultdict(set)
    expanded = set()
    tested_undefined = set()
    states = {}
    with TemporaryDirectory(prefix="Task0004-cgraph-") as tmp:
        directory = Path(tmp)
        for entry in project_units:
            unit = Path(entry["file"]).relative_to(sketch)
            command = entry["arguments"]
            output_index = command.index("-o")
            output = Path(command[output_index + 1])
            flags = [arg for arg in command[:output_index] + command[output_index + 2:]
                     if arg not in ("-c", "-MMD")]
            if Path(flags[0]).name != "avr-g++":
                raise ValueError(f"Unexpected compiler in {build}: {flags[0]}")
            working_dir = entry["directory"]
            traced = subprocess.check_output(
                [*flags, "-E", "-fdirectives-only", "-dI"], cwd=working_dir, encoding="utf-8", errors="replace")
            for path, lines in source_lines(traced, sketch, source).items():
                active[path].update(lines)
            macros = subprocess.check_output([*flags, "-E", "-dU"], cwd=working_dir,
                                             encoding="utf-8", errors="replace")
            for directive, name in MACRO.findall(macros):
                (expanded if directive == "define" else tested_undefined).add(name)

            dependencies = output.with_suffix(".d").read_text().replace("\\\n", "")
            for path in shlex.split(dependencies.split(": ", 1)[1]):
                path = Path(path)
                if path == sketch / unit:
                    continue
                if path.is_relative_to(sketch):
                    included["sketch"].add(path.relative_to(sketch).as_posix())
                elif "cores" in path.parts:
                    included["core"].add(Path(*path.parts[path.parts.index("cores") + 1:]).as_posix())
                elif "variants" in path.parts:
                    included["variant"].add(Path(*path.parts[path.parts.index("variants") + 1:]).as_posix())
                else:
                    included["other"].add(path.as_posix())

            obj = directory / unit.with_suffix(unit.suffix + ".o")
            obj.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run([*flags, "-c", "-fdump-ipa-cgraph", "-o", obj],
                           cwd=working_dir, check=True)
            dump = obj.with_suffix(".000i.cgraph")
            cgraph_states(dump.read_text(encoding="utf-8", errors="replace"), unit.as_posix(), states)
    return {
        "units": sorted(units),
        "dependencies": {kind: sorted(paths) for kind, paths in sorted(included.items())},
        "active": active,
        "expanded": expanded,
        "tested_undefined": tested_undefined,
        "states": [
            {"name": name, "scope": scope, **{key: sorted(uses.get(key, ()))
                                            for key in ("defined_in", "read", "write", "addr")}}
            for (name, scope), uses in sorted(states.items()) if uses.get("defined_in")
        ],
    }


def inventory_guards(source, active, strip):
    files = {}
    for path in sorted(source.rglob("*")):
        if path.suffix not in (".cpp", ".h", ".ino"):
            continue
        key = path.relative_to(source).as_posix()
        rows = strip(path.read_text(encoding="utf-8", errors="replace")).splitlines()
        statements = {number for number, row in enumerate(rows, 1)
                      if row.strip() and not CONDITION.match(row)}
        entries = []
        for start, end in sorted(branch_spans(rows)):
            body = statements.intersection(range(start + 1, end))
            if body:
                entries.append({
                    "line": start,
                    "directive": rows[start - 1].strip(),
                    "variants_emitting_body": [target for target, traces in active.items()
                                               if body.intersection(traces.get(key, ()))],
                })
        if entries:
            files[key] = entries
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("builds", type=Path, help="four-target evidence directory with Arduino build subdirectories")
    parser.add_argument("source", type=Path, help="Repetier directory from the checkout used for the builds")
    args = parser.parse_args()
    source = args.source.resolve()
    builds = args.builds.resolve()
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, text=True).strip()
    subprocess.run(["git", "diff", "--quiet", "HEAD", "--", "."], cwd=source, check=True)
    if (builds / "summary.json").exists():
        summary = json.loads((builds / "summary.json").read_text())
        if summary["sha"] != sha or set(summary["targets"]) != set(TARGETS) or summary["failures"]:
            raise ValueError("Build summary does not match a successful capture of this checkout")
    strip = runpy.run_path(str(Path(__file__).with_name("Task0004-scan.py")))["strip"]
    with ThreadPoolExecutor(max_workers=len(TARGETS)) as pool:
        pending = {target: pool.submit(inventory_target, builds / target / "build", source)
                   for target in TARGETS}
        targets = {target: future.result() for target, future in pending.items()}

    traces = {target: evidence.pop("active") for target, evidence in targets.items()}
    project_macros = set()
    for path in source.rglob("*"):
        if path.suffix in (".cpp", ".h", ".ino"):
            code = path.read_text(encoding="utf-8", errors="replace")
            project_macros.update(name for directive, name in MACRO.findall(strip(code))
                                  if directive == "define")
    for evidence in targets.values():
        expanded = evidence.pop("expanded")
        tested_undefined = evidence.pop("tested_undefined")
        evidence["project_macros_observed"] = sorted(project_macros & expanded)
        evidence["project_macros_tested_undefined"] = sorted(project_macros & tested_undefined)
        evidence["project_macros_not_observed"] = sorted(project_macros - expanded - tested_undefined)

    print(json.dumps({"sha": sha, "targets": targets, "guards": inventory_guards(source, traces, strip)},
                     indent=2, ensure_ascii=True))


if __name__ == "__main__":
    main()
