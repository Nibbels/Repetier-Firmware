# Bed-profile following, dynamic fade-out, ECMP and layer tracking

**Coordinate frames and physical geometry:** [how nominal slicer coordinates become the compensated Z surface, why valleys require additional height/material](coordinate-frames-and-extrusion.md).

## Bed-scan profile vs absolute offset

`BedCompensation::calculate()` takes the executing nominal Z, the planned layer and fade bounds, the matrix maximum and a reader for the local XY bed depth. It returns a Z correction in steps and the geometric extra extrusion per queued E step. Define `B(x,y)` as the local matrix correction and `O=g_offsetZCompensationSteps` as the common bed-profile offset.

| Height | Bed-profile behavior |
| --- | --- |
| `0 <= Z <= fadeStartSteps()` | Full XY-dependent scanned-bed correction to preserve nozzle gap |
| `fadeStartSteps() < Z < fadeEndSteps()` | Linearly fade XY-dependent variation toward baseline |
| `Z >= fadeEndSteps()` | Common bed-profile offset |

In the middle region the conceptual calculation from source is:

```text
C(x,y,Z) = O + (B(x,y)-O) * (Zmax-Z)/(Zmax-Zmin)
```

The matrix reader runs within the bed-profile region and during the last ECMP crossing. Later layers use the common offset. At `Z = 0` the full-profile region adds a 13-step clearance. Negative nominal Z uses the common offset plus that clearance. `recalculateZCompensation()` adds effective Z offset and active Digit-Z to the result and publishes the Z target and ECMP ratio together under interrupt protection. Physical endstop limits apply to the resulting motion.

## Why g_min and g_max are dynamic

All three models use nominal fallback settings of 0.33 mm and 10 mm for the start and end of the fade.

`PrintLayers` detects model layers as the planner queues moves.

1. A Z change proposes a candidate. Returning to the current layer clears the candidate.
2. Positive extrusion confirms the candidate and shifts the former current layer to the previous layer. The first XY extrusion at a positive Z can also establish a layer, including at the height of its `M3912` prime.
3. A descent within the first millimetre clears the previous layer. The first confirmed layer below 1 mm defines the fade start and the matrix relief defines its span.

```text
fade start = first model layer in steps
fade end = fade start + (maximum matrix value - minimum matrix value) * 20
```

The 2018 changelog describes this as spreading bed-profile correction out so it changes by no more than roughly **5% per layer**. Interpret this as an original engineering heuristic; actual outcome depends on real layer thickness and matrix values. Do not replace it with the 10 mm fallback unconditionally. Check that the computed fade interval stays valid and nonzero.

**Crucial:** this detector runs during **queue planning**. The pressure controller uses **current executed Z steps**. Resetting/changing shared boundaries at an inappropriate time can affect already executing segments.

## Geometric material compensation (ECMP)

Following an uneven first-layer surface then gradually printing flat upper layers alters the *physical layer thickness* at a given XY location. To maintain material deposition, the firmware computes an extra extrusion ratio from the interpolated matrix, `g_min`, `g_max` and the accepted current and previous layer heights. First and last layers crossing a fade boundary have special interpolation.

The foreground rounds the ratio to a multiple of 2⁻²³ and limits it to one extra pulse per base step. The timer adds the published ratio for each forward queued E step and subtracts it for each reverse step. Crossing a whole-step boundary emits or queues an extra step in that direction and leaves a remainder in `[0, 1)`. Equal opposite base moves at the same ratio therefore restore the original remainder and cancel all extra steps exactly. [ECMP accumulator](../../Repetier/ExtrusionCompensation.h)

Digit Flow Compensation reacts to DMS force and also affects material deposition.

## Interaction with M3912 and lifecycle

`M3912` resets layer tracking at entry and exit. Its phase flag bypasses the layer detector while queuing generated priming moves. Z homing also restores the default fade boundaries after disabling bed compensation. Disabling the Z stepper clears the confirmed current and previous layers.

An untested fork's RF2000v2-only reset-after-M3912 is not backed by model-specific differences in this shared algorithm. The *when* of resetting boundaries (particularly **before** priming) matters more than the model name.

## Source

- [Layer detector and dynamic boundaries](../../Repetier/PrintLayers.h)
- [Fade interpolation and material correction](../../Repetier/BedCompensation.h)
- [Offset composition and publication](../../Repetier/BedMatrix.cpp)
- [Stepper E compensation](../../Repetier/motion.cpp#L1093)
- [M3912 reset](../../Repetier/StartLine.cpp)
- [Z-stepper reset](../../Repetier/Printer.h#L355-L371)
- [RF1000](../../Repetier/RF1000.h) · [RF2000 family](../../Repetier/RF2000.h)
