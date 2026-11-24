#!/usr/bin/env python3
"""Read-only lexical audit of two immutable Git trees; no dead-code verdicts."""
import argparse
from collections import Counter
import json
from pathlib import PurePosixPath
import re
import subprocess

SOURCE = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".ino", ".s"}
LEX = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|'
                 r"'(?:\\.|[^'\\])*'", re.S)
WORD = re.compile(r'\b[A-Za-z_]\w*\b')
GUARD = re.compile(r'^\s*#\s*(?:if|ifdef|ifndef|elif|else|endif)\b')
MACRO = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)')
INCLUDE = re.compile(r'^\s*#\s*include\s*([<"][^>"]+[>"])')
STATIC = re.compile(r'\b(?:static|volatile|extern)\b.*[;=]')
WAIT = re.compile(r'\b(?:while\s*\(|delay(?:Milliseconds|Microseconds)?\s*\(|'
                  r'wait\s*\(|finishActive\s*\()')
FUNC = re.compile(r'^\s*([\w:*&<>,~ ]+?\b([\w:~]+))\s*\([^;{}]*\)'
                  r'\s*(?:const\s*)?(?:noexcept\s*)?\{')

def git(*args):
    return subprocess.check_output(["git", *args])

def read_tree(ref):
    sha = git("rev-parse", "--verify", ref + "^{commit}").decode().strip()
    paths = git("ls-tree", "-r", "--name-only", "-z", sha).decode().split("\0")
    paths = [p for p in paths if p]
    files = {p: git("show", sha + ":" + p) for p in paths
             if PurePosixPath(p).suffix.lower() in SOURCE and
             not p.startswith(("tests/", "coordination/"))}
    return sha, paths, files

def strip(text, literals=False):
    def replace(m):
        s = m.group()
        if s.startswith(("//", "/*")) or literals:
            return "".join("\n" if c == "\n" else " " for c in s)
        return s
    return LEX.sub(replace, text)

def normalized(text):
    return [" ".join(l.split()) for l in strip(text).splitlines() if l.strip()]

def shingles(lines, n=5):
    return Counter(tuple(lines[i:i+n]) for i in range(len(lines)-n+1))

def functions(code):
    # Single-line signatures only; conditionals and macro-heavy functions may
    # be skipped or mis-sized. Braces in comments and literals are masked.
    rows = code.splitlines()
    for i, line in enumerate(rows):
        m = FUNC.match(line)
        if not m or m.group(2) in {"if", "for", "while", "switch", "catch"}:
            continue
        depth = 0
        for j in range(i, len(rows)):
            s = rows[j][m.end()-1:] if j == i else rows[j]
            depth += s.count("{") - s.count("}")
            if depth <= 0:
                yield [m.group(2), i+1, j+1, j-i+1]
                break

def scan(files, old_files):
    all_words = Counter()
    references = {}
    old_lines = Counter()
    old_sequences = Counter()
    for raw in old_files.values():
        lines = normalized(raw.decode("utf-8", "replace"))
        old_lines.update(lines)
        old_sequences.update(shingles(lines))
    texts = {p: raw.decode("utf-8", "replace") for p, raw in files.items()}
    for p, t in texts.items():
        words = Counter(WORD.findall(strip(t, literals=True)))
        all_words.update(words)
        for w, count in words.items():
            references.setdefault(w, {})[p] = count
    result = {}
    for p, t in texts.items():
        code = strip(t, literals=True)
        clean = strip(t)
        rows = code.splitlines()
        lines = normalized(t)
        prev = normalized(old_files.get(p, b"").decode("utf-8", "replace"))
        same = Counter(lines) & Counter(prev)
        macros = [(i+1, m.group(1), all_words[m.group(1)])
                  for i, row in enumerate(rows) if (m := MACRO.match(row))]
        fn = sorted(functions(code), key=lambda x: (-x[3], x[1]))
        # Include candidate refs by spelling (NOT resolved readers/writers).
        state = []
        for i, row in enumerate(rows):
            if STATIC.search(row) and not row.lstrip().startswith("#"):
                names = {w: references[w] for w in WORD.findall(row)
                         if w in references and w not in
                         {"static", "volatile", "extern", "const"}}
                state.append([i+1, row.strip(), names])
        result[p] = {
            "bytes": len(files[p]), "physical_lines": len(rows),
            "normalized_lines": len(lines),
            "same_path_line_multiset_matches": sum(same.values()),
            "old_any_path_line_multiset_matches": sum((Counter(lines) & old_lines).values()),
            "old_any_path_5line_matches": sum((shingles(lines) & old_sequences).values()),
            "functions_heuristic": fn,
            "function_names_low_reference_count": [
                [name, start, all_words[name.split("::")[-1]]]
                for name, start, _, _ in fn if all_words[name.split("::")[-1]] <= 2],
            "static_volatile_extern_candidates": state,
            "macro_definitions_with_token_count": macros,
            "includes": [[i+1, m.group(1)] for i, row in enumerate(clean.splitlines())
                         if (m := INCLUDE.match(row))],
            "preprocessor_guards_unresolved": [[i+1, row.strip()] for i, row in enumerate(rows)
                                               if GUARD.match(row)],
            "wait_candidates": [[i+1, row.strip()] for i, row in enumerate(rows) if WAIT.search(row)],
            "stale_comment_candidates": [[i+1, row.strip()] for i, row in enumerate(t.splitlines())
                                         if re.search(r'\b(?:TODO|FIXME|XXX|SdFat|SDCard|u8glib)\b', row)]
        }
    duplicate = {}
    for p, t in texts.items():
        for seq, count in shingles(normalized(t)).items():
            if count:
                duplicate.setdefault(seq, []).append([p, count])
    return result, [
        {"lines": seq, "locations": places} for seq, places in duplicate.items()
        if sum(n for _, n in places) > 1]

def self_test():
    a = '// comment\nint f() {\n  static int x = 0;\n  return x; /* z */\n}\n'
    assert normalized(a) == ['int f() {', 'static int x = 0;', 'return x;', '}']
    assert list(functions(strip(a, True))) == [['f', 2, 5, 4]]
    assert 'http://x' in strip('const char* s = "http://x";')
    assert 'http' not in strip('const char* s = "http://x";', True)
    f, d = scan({"Repetier/A.cpp": a.encode()}, {"Repetier/Old.cpp": a.encode()})
    assert f["Repetier/A.cpp"]["same_path_line_multiset_matches"] == 0
    assert f["Repetier/A.cpp"]["old_any_path_line_multiset_matches"] == 4
    assert f["Repetier/A.cpp"]["static_volatile_extern_candidates"][0][0] == 3
    assert not d
    print("Task0004 lexical fixture tests passed")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base")
    parser.add_argument("--head")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.base or not args.head:
        parser.error("--base and --head required")
    b, bp, bf = read_tree(args.base)
    h, hp, hf = read_tree(args.head)
    per_file, duplicates = scan(hf, bf)
    print(json.dumps({
        "base": b, "head": h, "source_files": [len(bf), len(hf)],
        "source_bytes": [sum(map(len, bf.values())), sum(map(len, hf.values()))],
        "removed_paths": sorted(set(bp) - set(hp)),
        "added_paths": sorted(set(hp) - set(bp)),
        "unchanged_source_paths": sorted(p for p in bf if bf[p] == hf.get(p)),
        "build_library_paths": sorted(p for p in hp if re.search(
            r'cmake|vcxproj|library|libraries|arduino|\.sln$|\.github/', p, re.I)),
        "files": per_file, "duplicate_5line_windows": duplicates,
        "warning": "Lexical candidates only; no preprocessing, call graph or linked byte evidence."
    }, indent=2, ensure_ascii=True))

if __name__ == "__main__":
    main()
