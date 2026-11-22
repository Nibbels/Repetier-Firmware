# Bed-profile following, dynamic fade-out, ECMP and layer tracking

**Coordinate frames and physical geometry:** [how nominal slicer coordinates become the compensated Z surface, why valleys require additional height/material](coordinate-frames-and-extrusion.md).

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

All three models use nominal fallback settings of 0.33 mm and 10 mm for the start and end of the fade.

`PrintLine::prepareQueueMove()` detects a prospective layer:
1. A Z change sets `queuePositionZLayerGuessNew`.
2. A subsequent positive E move accepts it into `queuePositionZLayerCurrent` and moves the previous current to `Last`.
3. When a new layer is below the old one and within the first millimetre, `Last` may reset to zero (including sequential prints).
4. When `Last==0` and the candidate is <1 mm, the firmware can make `g_min` equal the **real first layer height**.
5. With no tracked layer or Z candidate, an XY move with positive extrusion can establish a layer at the current positive Z. This covers a model that starts at the height of its `M3912` prime.
6. The planner clears `GuessNew` after processing positive extrusion.

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

`M3912` clears all three layer markers at entry and exit. Its phase flag bypasses the layer detector while queuing generated priming moves. Z homing also restores the default `g_min/g_max` after disabling bed compensation. Disabling the Z stepper clears `Current` and `Last`.

An untested fork's RF2000v2-only reset-after-M3912 is not backed by model-specific differences in this shared algorithm. The *when* of resetting boundaries (particularly **before** priming) matters more than the model name.

## Source
- [Layer detector and dynamic boundaries](../../Repetier/motion.cpp#L170-L203)
- [Fade interpolation, material correction, offsets](../../Repetier/RF.cpp#L3192-L3302)
- [Stepper E compensation](../../Repetier/motion.cpp#L1385-L1500)
- [M3912 reset](../../Repetier/RF.cpp#L9288-L9313)
- [Z-stepper reset](../../Repetier/Printer.h#L355-L371)
- [RF1000](../../Repetier/RF1000.h) · [RF2000 family](../../Repetier/RF2000.h)
