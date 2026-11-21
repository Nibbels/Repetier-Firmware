# Bed-profile following, dynamic fade-out, ECMP and layer tracking

## Bed-scan profile vs absolute offset

`recalculateHeatBedZCompensation()` reads the interpolated XY bed matrix and calculates an additional Z motor target. Define `B(x,y)` as matrix correction and `O=g_offsetZCompensationSteps` as the common compensation offset. There are three regions in **nominal Z motor steps**:

| Height | Bed-profile behavior |
| --- | --- |
| `Z <= g_minZCompensationSteps` | Full XY-dependent scanned-bed correction to preserve nozzle gap |
| `g_minZCompensationSteps < Z < g_maxZCompensationSteps` | Linearly fade XY-dependent variation toward baseline |
| `Z >= g_maxZCompensationSteps` | No *spatial* variation remains; common offset persists |

In the middle region the conceptual calculation from source is:

```text
C(x,y,Z) = O + (B(x,y)-O) * (Zmax-Z)/(Zmax-Zmin)
```

Additionally applied: `g_staticZSteps`, optional force-dependent Digit-Z correction, and physical endstop limits. Therefore 'zero compensation above max' would be misleading: **variable bed shape ends, not all Z offsets**.

## Why g_min and g_max are dynamic

All three models currently declare the same nominal fallback settings: `MIN_MM=0.33`, `MAX_MM=10`, special priming exclusion `AUTOADJUST_STARTMADEN_AUSSCHLUSS=0.352`.

`PrintLine::prepareQueueMove()` detects a prospective layer:
1. A Z change sets `queuePositionZLayerGuessNew`.
2. A subsequent positive E move accepts it into `queuePositionZLayerCurrent` and moves the previous current to `Last`.
3. When a new layer is below the old one and within the first millimetre, `Last` may reset to zero (including sequential prints).
4. When `Last==0` and the candidate is <1 mm, the firmware can make `g_min` equal the **real first layer height**.
5. It excludes Z values within **5 steps** of `0.352 mm` so priming does not set those boundaries.
6. `GuessNew` is cleared after positive extrusion.

The dynamic fade limit is:

```cpp
g_minZCompensationSteps = Printer::queuePositionZLayerCurrent;
g_maxZCompensationSteps =
    g_minZCompensationSteps +
    (g_offsetZCompensationSteps - g_ZCompensationMax) * 20;
```

The 2018 changelog describes this as spreading bed-profile correction out so it changes by no more than roughly **5% per layer**. Interpret this as an original engineering heuristic; actual outcome depends on real layer thickness and matrix values. Do not replace it with the 10 mm fallback unconditionally. Check that the computed fade interval stays valid and nonzero.

**Crucial:** this detector runs during **queue planning**. The pressure controller uses **current executed Z steps**. Resetting/changing shared boundaries at an inappropriate time can affect already executing segments.

## Geometric material compensation (ECMP)

Following an uneven first-layer surface then gradually printing flat upper layers alters the *physical layer thickness* at a given XY location. To maintain material deposition, the firmware computes a layer-dependent extrusion correction (`nNeededZEPerc`) from the interpolated matrix, `g_min`, `g_max` and accepted current/previous layer heights. This becomes `Printer::compensatedPositionOverPercE`; the stepper path accumulates fractional E steps and inserts appropriate extra steps. First and last layers crossing a fade boundary have special interpolation.

**ECMP is not pressure-based Digit Flow Compensation.** One changes extruded material according to geometric layer changes; the other reacts to DMS force. Both can affect final E deposition, so changing fade boundaries requires examining ECMP as well.

## Interaction with M3912 and lifecycle

The priming line at Z=0.352 mm can pass through the layer detector, but that exact height is excluded from **dynamic boundary learning**. `M3912` subsequently clears `queuePositionZLayerCurrent` and `Last`; it does **not** itself reset `GuessNew` or restore `g_min`/`g_max`. **Z homing now clears all three layer markers and restores min/max after disabling CMP.** Z-stepper disable alone still clears only `Current`/`Last`.

An untested fork's RF2000v2-only reset-after-M3912 is not backed by model-specific differences in this shared algorithm. The *when* of resetting boundaries (particularly **before** priming) matters more than the model name.

## Source
- [Layer detector and dynamic boundaries](../../Repetier/motion.cpp#L173-L219)
- [Fade interpolation, material correction, offsets](../../Repetier/RF.cpp#L3192-L3302)
- [Stepper E compensation](../../Repetier/motion.cpp#L1385-L1500)
- [M3912 reset](../../Repetier/RF.cpp#L9245-L9253)
- [Z-stepper reset](../../Repetier/Printer.h#L355-L371)
- [Same three machine configurations](../../Repetier/RF1000.h#L762-L773) · [RF2000](../../Repetier/RF2000.h#L769-L780) · [RF2000v2](../../Repetier/RF2000v2.h#L769-L780)
