#!/usr/bin/env python3
"""Check that both Visual Micro manifests track the complete firmware sketch."""
from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET

SKETCH = Path(__file__).resolve().parents[1] / "Repetier"
NS = "{http://schemas.microsoft.com/developer/msbuild/2003}"
GENERATED_HEADER = r"__vm\.Repetier.vsarduino.h"


def check_manifest(path, filters):
    root = ET.parse(path).getroot()
    for tag, suffix, expected_filter in (
        ("ClCompile", ".cpp", "Source Files"),
        ("ClInclude", ".h", "Header Files"),
    ):
        entries = [node for node in root.iter(NS + tag) if "Include" in node.attrib]
        actual = Counter(node.attrib["Include"] for node in entries)
        expected = Counter(p.name for p in SKETCH.glob("*" + suffix) if p.is_file())
        if tag == "ClInclude":
            # Visual Micro's IDE-generated IntelliSense header is not a sketch input.
            expected[GENERATED_HEADER] = 1
        assert actual == expected, (
            f"{path.name} {tag}: missing={expected - actual}, "
            f"unexpected/duplicate={actual - expected}"
        )
        if filters:
            for node in entries:
                assert node.findtext(NS + "Filter") == expected_filter, node.attrib
    sketches = [node.attrib["Include"] for node in root.iter(NS + "None")
                if node.attrib.get("Include", "").endswith(".ino")]
    assert sketches == ["Repetier.ino"], (path.name, sketches)
    print(f"PASS {path.name}: {len(list(SKETCH.glob('*.cpp')))} C++ sources, "
          f"{len(list(SKETCH.glob('*.h')))} sketch headers")


if __name__ == "__main__":
    check_manifest(SKETCH / "Repetier.vcxproj", False)
    check_manifest(SKETCH / "Repetier.vcxproj.filters", True)
