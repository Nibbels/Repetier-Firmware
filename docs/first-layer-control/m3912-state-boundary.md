# M3912: implemented pressure-braked priming transaction

> **Status:** [Firmware patch 5f848c9](https://github.com/Nibbels/Repetier-Firmware/commit/5f848c9a5cd0347c6d45fd9d3f13afd7344cb775) and [atomic timer cleanup b793cd8](https://github.com/Nibbels/Repetier-Firmware/commit/b793cd8203e7ed71154728412a03d4e9f4d7fb67) on `ai-firmware-review`; source and arithmetic checks passed. **An AVR build and a physical printer test have not been performed.**

## Intent: DMS-controlled braking, not insufficient motor power

The operator's **chainsaw-under-load analogy** describes the audible effect: while a chainsaw slows under wood load, here the printer **deliberately applies a simulated, gradually increasing brake** when DMS detects high extrusion force. The motors are not necessarily at their power limit. The firmware primarily slows XY/filament throughput to relieve resistance, and additionally reduces extrusion amount. This is the original intended pressure-response mode for `M3912`; it is fundamentally different from changing permanent or dynamic Z clearance.

Default thresholds are **1500–4500 DMS digits** (overridable by `M3912 S/P`), with speed intensity −90% and extrusion intensity −75%. At high force the target speed factor reaches **0.10** and the E factor **0.25**. The speed factor approaches its target by small 0.01 steps per update; planned E segments use the instantaneous E factor. These are not direct pressure measurements.

**Never change the user's normal M220 speed override** (`Printer::feedrateMultiply`) or M221 flow multiplier (`Printer::menuExtrusionFactor`). The DMS speed factor is `Printer::dynamicFeedrateFactor`, applied through `Printer::interval_mod` to live XY-primary step timing. The DMS E factor is `Printer::dynamicExtrusionFactor`, applied during E planning.

## Deliberate Z=0.352 mm sentinel

The unusual **0.352 mm** start height was intentionally chosen above typical first-layer heights and unlikely to match a slicer's first-layer Z. The legacy planner exclusion (±5 Z steps) remains intact for manually generated start lines. A real model layer exactly at that special height is the acknowledged rare collision.

The firmware-generated `M3912` now also uses a temporary `g_startLineActive` flag:
- During priming, enable DMS feed/E braking **regardless of stale `g_min`** from earlier prints.
- Suppress **new SenseOffset increments**, while preserving the already applied positive correction and the EEPROM-backed manual Z-offset.
- Keep priming movement out of the first-model-layer detector, and clear its pressure sample history.
- After the prime queue has completely finished, clear the flag and permit regular first-layer learning.

This additional signal does **not** reset the bed matrix or stale `g_min/g_max` geometry. Starting a second print without Z homing can still leave the old spatial fade thresholds; see [Z homing lifecycle](../z-offset/lifecycle-homing-fix.md).

## Cache/restore ledger

| State | Action at entry | Action at exit |
| --- | --- | --- |
| `g_nDigitFlowCompensation_Fmin/Fmax` | Save, override with 1500/4500 or S/P | Restore |
| DMS speed and extrusion intensities | Save, set −90% and −75% | Restore |
| M82/M83 E-coordinate mode | Save, force absolute | Restore |
| Logical slicer E coordinate | Save incoming value, use local E0 for prime | Restore incoming E unless print was cancelled |
| `extrudeMultiplyErrorSteps` | Save planner sub-step E remainder | Restore on normal/skip completion |
| `compensatedPositionCollectTinyE` | Save ECMP fractional E remainder under interrupt protection | Restore on normal/skip completion |
| DMS `dynamicFeedrateFactor` and `dynamicExtrusionFactor` | Continuously adapt to DMS force | Reset both to 1.0 |
| DMS `interval_mod` | Continuously computed from DMS speed | Reset to 0 atomically; no lingering XY brake |
| `queuePositionZLayerGuessNew/Current/Last` | Clear and exclude prime movement from learning | Clear all three |
| `g_startLineActive` | Set before first prime move | Clear after queue drain |
| User `feedrateMultiply`, `menuExtrusionFactor`, `Printer::feedrate` | **Never modified** | Unchanged |
| `Printer::ZOffset`, existing SenseOffset, matrix, `g_min/g_max`, XY/Z position | **Preserve** | Unchanged by reset logic |

**Logical E example:** if the slicer has already reached absolute **E10**, the internal prime can physically push many additional millimetres of filament using its own temporary E0-based coordinates. Afterwards the firmware **restores logical E10**; the next slicer `E11` requests only 1 mm. With conventional entry E0, restoring E0 behaves like the original start code. On cancellation the stop handler's intentional E0 reset is preserved; play-key skip is treated as successful cleanup without extra Y travel.

## What the change deliberately does not touch

The original DMS-to-feed/E control equation, E intensity and step-based XY speed modulation are unchanged. Bed Z compensation continues to follow the 2D matrix and fade as before; at Z0.352 and baseline g_min0.33 the line is 22 µm into the fade. Geometric ECMP and **instantaneous Digit-Z force correction** remain active as before. They are distinct from accumulated SenseOffset. No additional EEPROM writes, automatic Z-offset transfers or user speed changes were introduced.

If `FEATURE_STARTLINE` is compiled but `FEATURE_DIGIT_FLOW_COMPENSATION` is disabled, M3912 emits an explicit error instead of invoking unavailable Digit Flow settings or running unbraked priming.

## Tests performed and remaining tests

**Static/source checks passed:** active flag before queued moves and cleared after queue drain, SenseOffset disabled only during generated prime, Digit Flow bypass of stale first-layer gate, numeric 0.352 sentinel preserved, all three layer markers cleared, no user speed/flow assignments, E reference restored, fractional E state preserved, and no extra Y move after skip.

**Arithmetic cases passed:** E10→M3912→E11 requests +1 E; E0→M3912→E1 requests +1 E. At DMS 1500 the speed/E factors are 1.0/1.0, at 3000 the speed **goal**/E factor are 0.55/0.625, and at ≥4500 they are 0.10/0.25. Actual speed ramps toward the goal rather than jumping.

**Required before production:** AVR builds for RF1000, RF2000 and RF2000v2, high-force/low-force real priming, live user M220/M221 overrides, M82/M83 and absolute/nonzero E cases, play-key skip and print cancellation, first model layers at 0.20 and 0.40 mm, and consecutive prints with/without Z homing. Verify the physical speed/force response audibly and via logging without assuming high DMS means insufficient motor torque.

## Source

[M3912, the DMS controller and SenseOffset](../../Repetier/RF.cpp) · [first-layer planner](../../Repetier/motion.cpp) · [shared priming flag](../../Repetier/RF.h) · [coordinate frames and ECMP](coordinate-frames-and-extrusion.md).
