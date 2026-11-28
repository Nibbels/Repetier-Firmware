#!/usr/bin/env python3
"""The hard filament programs now execute typed operations, not strings."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]/"Repetier"
for board in ("RF1000","RF2000"):
    source=(root/(board+".h")).read_text()
    assert not re.search(r"#define\s+(?:UN)?MOUNT_FILAMENT_SCRIPT_HARD\b",source)
source=(root/"GCodeInput.cpp").read_text()
assert "unload ? PREHEAT_A_HOTEND_C : PREHEAT_B_HOTEND_C" in source
assert "internalAxisMove(E_AXIS, unload ? -90.0f : 90.0f, unload ? 500.0f : 100.0f);" in source
ui=(root/"UiActions.cpp").read_text()
assert ui.count("GCodeInput::executeHardFilament(true);")==1
assert ui.count("GCodeInput::executeHardFilament(false);")==1
print("Hard filament typed operation contract PASS; actual execution replay in serial-protocol")
