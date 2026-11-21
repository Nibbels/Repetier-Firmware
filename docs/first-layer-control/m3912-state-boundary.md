` M3912 priming: state transaction design (not yet implemented)

This design review applies to RF1000/RF2000/RF2000v2 shared code on `ai-firmware-review` as of November 2026. See [current automatic start line](automatic-start-line.md) and [implemented Z-homing reset](../z-offset/lifecycle-homing-fix.md). **The following M3912 changes remain proposals; no start-line firmware code was modified.**

`` Rule: cache configuration, discard priming state, preserve physical offsets

| State | Before prime | After prime | Decision |
| --- | --- | --- | --- |
| `g_nDigitFlowCompensation_Fmin/Fmax` | Cache, override with 1500/4500 digits or M3912 S/P | Restore caller values | Already implemented correctly |
| `g_nDigitFlowCompensation_intense` | Cache; set −75% flow | Restore | Already implemented |
| `g_nDigitFlowCompensation_speed_intense` | Cache; set −90% speed | Restore | Already implemented |
| `Printer::relativeExtruderCoordinateMode` (M82/M83) | Cache, force absolute mode | Restore | Already implemented in normal feature configuration |
| E-coordinate reference | `setEAxisSteps(0)` | `setEAxisSteps(0)` | Intentional **G92 E0** contract, not a save/restore; changing it requires slicer compatibility checks |
| `Printer::dynamicExtrusionFactor` / `dynamicFeedrateFactor` | Force-adjusted during prime | Set 1.0 | Correct: **discard derived prime response** rather than restoring stale pre-prime pressure |
| `Printer::interval_mod` (step timing) | Force-derived during prime | **Not explicitly cleared** | **Cleanup gap**: neutralize after queue drain to prevent prime slowdown affecting next XY move |
| `queuePositionZLayerLast/Current` | Can be contaminated by prime Z=0.352 | Both reset | Keep, and ideally exclude M3912 from layer detection altogether |
| `queuePositionZLayerGuessNew` | Can hold priming Z move | **Not reset** | Clear at start/end, including on early skip |
| `g_minZCompensationSteps / g_maxZCompensationSteps` | Should be defaults after fresh G28 Z | Unchanged | **Never restore old learned bounds from snapshot**; printing without homing remains a separate unresolved lifecycle case |
| `g_nSensiblePressureOffset` and saved `Printer::ZOffset` | Already-applied physical clearance / operator transfer | Must remain | **Do not cache/restore or zero them** merely because M3912 runs |
| Matrix/slot/temperatures/tool offsets, XYZ position | Normal print setup | Normal print setup | **Leave untouched** (XYZ intentionally ends at prime line) |
| `Printer::feedrate`, G90/G91, G20/G21, menu flow multiplier | Not modified by M3912 | Unmodified | No need to cache |

`` Proposed M3912 lifecycle

**Entry:** drain prior queued moves; verify homed axes and enabled bed compensation. For the normal print sequence `G28 Z → M3001 → M3912`, homing has already reset learned min/max and dynamic SenseOffset. Start a short-lived **explicit priming phase** before the first priming move, and temporarily apply special Digit Flow config. While this phase is active, **prevent SenseOffset from incrementing** regardless of nominal Z, but do not erase already-applied clearance.

**During prime:** require pressure-based Digit Flow (less E and slower XY) instead of relying solely on the `currentZ > g_min` heuristic. As currently written, if a previous print left `g_min=0.40 mm` without rehoming, priming at 0.352 mm can incorrectly allow SenseOffset and block Digit Flow. An explicit phase flag can isolate the **pressure-control policy**; it does not automatically repair stale geometric fade bounds for no-homing prints. Exclude M3912-generated moves from the first-model-layer detector; the numeric 0.352 mm exclusion can remain for legacy manual start lines.

**Exit:** wait for or abort remaining priming moves. Restore all four temporary Digit Flow settings and original extruder-coordinate mode through a single cleanup path (normal / play-key skip / print cancel). Neutralize derived speed/extrusion factors **and `interval_mod`**; clear all three layer markers (especially `GuessNew`). Drop any priming-specific SenseOffset pressure averages so real first-layer pressure control begins with fresh samples; then clear the priming phase flag. Do not restore obsolete layer-boundary snapshots or physical Z offsets.

`` Source-backed issues to test

1. **Missing `GuessNew` reset:** M3912 currently resets `Last` and `Current` only. If play skips before first extrusion, the Z travel can leave an unaccepted guess.
2. **Stale timer modifier:** at exit the code sets `dynamicFeedrateFactor=1` but does not set `interval_mod=0`; the timer uses the latter for XY slowdown until a strain-gauge update recomputes it.
3. **Play-key skip can queue another Y move** after exiting the segment loop. No new priming travel should be queued following an explicit skip or stop.
4. **Compile guard inconsistency:** M3912 declares/copies `g_nDigitFlowCompensation_*` even though that configuration can be compiled off while `FEATURE_STARTLINE` remains enabled. Check alternative feature builds.
5. **Extruder-coordinate semantics:** internal `queueFloatCoordinates()` uses explicit absolute E coordinates regardless of G90/G91 and M82/M83, but M3912 changes the M82/M83 flag and also issues implicit G92 E0 twice. Do not change that contract without checking slicer start code.
6. **Pressure history:** current SenseOffset low-Z exit clears samples only if pressure sum is >0; negative DMS force can leave stale history. Use a priming-specific fresh-history policy rather than relying on that condition.
7. **No-homing print:** the G28 Z lifecycle reset solves the ordinary start-code sequence. M3912 alone is not necessarily a safe moment to zero positive SenseOffset or redefine faded Z compensation while the nozzle is close to the bed.

`` Tests before a code patch

- Normal `G28 Z → M3001 → M3912` with SenseOffset autostart and high DMS: **Digit Flow adapts**, SenseOffset never increases during prime.
- First model layers Z0.20 and Z0.40: fresh layer detection, full bed shape and correct fade/ECMP after priming.
- Skip before first positive extrusion; skip midway; stop/cancel with moves queued: no extra movements, all configs restored, all three markers cleared.
- Custom M3911 settings before prime: exact restoration, normal speed after prime (no stale `interval_mod`), valid pressure response on subsequent model extrusion.
- M83 and G91 modes; E-axis logical zero semantics and slicer first extrusion compatibility.
- Saved manual Z transfer performed with Mod Menu: remains valid; no rollback of positive RAM correction during priming.
- Compile with `FEATURE_STARTLINE=1` and `FEATURE_DIGIT_FLOW_COMPENSATION=0` or explicitly reject that combination.
- Consecutive prints with and without `G28 Z`; a negative-polarity DMS sensor.

`` Source

[M3912](../../Repetier/RF.cpp`L9103-L9275) · [Digit Flow factor and interval](../../Repetier/RF.cpp`L315-L398) · [SenseOffset](../../Repetier/RF.cpp`L5623-L5780) · [Layer detector](../../Repetier/motion.cpp`L173-L220) · [Timer consumer](../../Repetier/motion.cpp`L1645-L1654) · [Direct coordinate queue](../../Repetier/Printer.cpp`L522-L544).
