# Coordinate frames, bed-profile fade-out and the extrusion timeline

> Verified against the shared source for RF1000/RF2000/RF2000v2 on `ai-firmware-review`, 2026-11-21. Conceptual source review plus small arithmetic checks; **not a firmware simulation or physical test**.

## Z: four different layers of meaning

1. **Theoretical slicer G-code**: `G1 Z0.20` requests a nominal layer height in the G-code's current work origin, G90/G91 mode and units. This number is not necessarily physical Z motor steps.
2. **Uncompensated planned firmware position**: `Printer::queueGCodeCoordinates()` converts units and uses relative or absolute positioning; in absolute G-code mode it subtracts `Printer::originOffsetMM[Z_AXIS]`. `Printer::destinationMM[Z_AXIS]` is then mapped by `getDestinationSteps(Z_AXIS) = lroundf(destinationMM[Z]*axisStepsPerMM[Z])`. The planner later executes this value as `Printer::currentSteps[Z_AXIS]`. **The learned `g_min`/`g_max` boundaries live in this nominal planned step-coordinate frame.** Normally with zero G-code origin shift, they numerically match the slicer's Z height.
3. **Compensation/Z motor target**: `recalculateHeatBedZCompensation()` uses the *executing nominal* `currentSteps[Z_AXIS]` to select the height-dependent shape compensation and adds `g_staticZSteps` (menu Z + accumulated SenseOffset). A separate optional **Digit-Z** force correction and endstop rules may also apply. Target is `compensatedPositionTargetStepsZ`; actual motor correction `compensatedPositionCurrentStepsZ` can lag behind the target.
4. **Real nozzle-to-bed clearance and printed geometry**: combines nominal Z motion, actual compensation motor steps, tool offsets, physical thermal drift and the real bed surface. This is not directly identical to any one planned variable.

The source intentionally avoids including extruder-specific Z displacement in the nominal fade-height coordinate. Both the layer detector and fade calculation reason in the nominal planned axis, even though extruder/tool offsets or manual direct moves can change real nozzle clearance.

## What `g_min` and `g_max` really mean

`motion.cpp::PrintLine::prepareQueueMove()` accepts a prospective first layer after a Z move followed by **positive physical E steps** in the queue. It records `Printer::queuePositionZLayerCurrent`, then:

```cpp
g_minZCompensationSteps = Printer::queuePositionZLayerCurrent;
g_maxZCompensationSteps = g_minZCompensationSteps
    + (g_offsetZCompensationSteps - g_ZCompensationMax) * 20;
```

- **`g_min` = actual first model layer's nominal planned Z** (not the distance from the *physically uneven* bed at every XY point). Full bed-shape following extends through that nominal layer.
- **`g_max` = the nominal Z at which XY-dependent bed-shape variation has completely faded**. It depends on matrix relief, not a fixed number of model layers.
- These are **boundaries in the *input* nominal Z frame**, not Z corrections themselves or already-compensated physical positions. Their values are stored in motor **steps**, not mm or raw G-code strings.
- The default 0.33 mm/10 mm are fallbacks; the first model layer dynamically replaces the lower limit and the formula sets the upper.
- The current special 0.352 mm exclusion is numeric; a true model layer at that height can be misclassified. A future explicit priming phase would be more reliable.

### Why the model can become physically higher in bed valleys

Let `B(x,y)` be the interpolated matrix correction and `O = g_offsetZCompensationSteps = max(matrix values)`. The code also stores `g_ZCompensationMax = min(matrix values)`. Because `B <= O`, fading from `B` to `O` makes the correction at local bed valleys increase toward the common flat plane:

```text
C(x,y,z) =
  B(x,y)                                          z <= g_min
  O + (B(x,y)-O)*(g_max-z)/(g_max-g_min)           g_min < z < g_max
  O                                               z >= g_max
```

Example with 2560 Z steps/mm, first layer 0.20 mm, matrix minimum −100 steps and maximum +100 steps:
- `g_min=512 steps` (=0.20 mm).
- `g_max=512+(100-(-100))*20=4512 steps` (1.7625 mm), so fade span is 1.5625 mm.
- At a bed location `B=-100`, the correction progresses from −100 at first layer through 0 halfway to +100 above max. **The top above this local bed point is lifted by 200 steps ≈0.0781 mm relative to the original local bed-following plane.**
- At `B=O=+100`, no such extra local lift is needed. The resulting upper print surface is level relative to the chosen common matrix reference.

This validates the user's qualitative point: **printing flat layers over a locally depressed bed area can require additional deposited material and extra height relative to the local bed**. It does *not* mean that every dimension or every XY location universally grows by the same absolute amount: it depends on how the matrix is referenced, Z-offsets, and the local bed relief.

### Why ECMP exists

As `C(x,y,z)` changes during fade-out, the true vertical separation between successive deposited layers is not always just the slicer's nominal layer height. The firmware's *geometric E material compensation* (`Printer::compensatedPositionOverPercE`) inserts extra E stepping so that these stretched layers are filled instead of under-extruded. In the example the nominal fade slope is 200 / 4000 = **0.05 (5%)**. This is why the original `*20` factor was chosen. The actual code includes special handling at the first and last fade boundaries.

**Important:** ECMP is geometric and independent of **Digit Flow**, which adjusts E volume in response to extrusion force. Do not mix ECMP, live SenseOffset, or instantaneous Digit-Z corrections when changing the fade equation.

## E: logical coordinate is not physical filament pushed

For absolute-E G-code, the slicer has a **logical E timeline** (for example, 10 → 11), while the firmware planner calculates *actual E steps* from successive internal logical destinations. `PrintLine::prepareQueueMove()` multiplies each E delta by `menuExtrusionFactor * dynamicExtrusionFactor`, accumulates step rounding, then ECMP can add more E steps as needed. Physical extruded filament thus need not equal the logical numeric difference exactly.

`Printer::setEAxisSteps(...)` changes **only the logical E coordinate pair** `destinationMM[E_AXIS]` and `destinationMMLast[E_AXIS]`. It does **not** unextrude filament or reset an actual counted E motor position. This is precisely the operation needed for firmware-generated priming.

### Arithmetic test: restore the logical timeline

Suppose the slicer is at absolute `E10`, then calls `M3912`, whose internal priming commands end near logical `E50`. The slicer's next move is `E11`.

| End of M3912 | Next slicer E11 interpreted as | Result |
| --- | --- | --- |
| Current implementation: force logical E0 | +11 E units | **Wrong if M3912 was called after E10** |
| Proposed: restore entry logical E10 | +1 E unit | **Correct** |
| Conventional start script with pre-prime E0 | +11? No: typically next slicer E starts near 0 | Existing reset works for this restricted case |

**Design recommendation:** after the entry queue drain, snapshot the *logical* `destinationMM[E_AXIS]`, use a local zero-based E timeline for priming, then restore the snapshotted logical E **after all priming moves have completed**. This is backward-compatible with the normal E0 case, and handles arbitrary nonzero entry coordinates. Restore the M82/M83 flag as before. Careful atomic handling and step-vs-float precision should be verified before implementing.

In relative E mode, firmware-generated absolute E coordinates remain internal to `queueFloatCoordinates()`; restoring logical E is still harmless and preserves later G-code interpretation. Retracts cause occasional decreases in the logical E timeline, but do not invalidate this model.

## DMS speed is orthogonal to user speed override

- `Printer::feedrateMultiply` (typically M220 or menu speed%) changes the user's normal requested feedrate. The stored `Printer::feedrate` incorporates that multiplier.
- `Printer::dynamicFeedrateFactor` is a separate DMS-based value (1.0 means neutral). It is translated to `Printer::interval_mod`, a timer divisor applied at **execution time** to XY-primary queued moves; it **does not modify** `feedrateMultiply`.
- `Printer::menuExtrusionFactor` (user flow%) and `Printer::dynamicExtrusionFactor` (DMS flow) likewise remain separate; the planner multiplies them when generating physical E steps.
- `M3912` uses explicit mm/s for its internal queue moves; it does not change or save the general user speed override. Consequently, don't assume that the start-line's configured XY feedrate always applies M220 in exactly the same way as a slicer G1 F move. Pressure-derived step slowdown remains an additional mechanism.
- Returning `dynamicFeedrateFactor` to 1.0 is correct **but derived `interval_mod` should also be neutralized** after priming to avoid a stale timer-scale effect.

## Another DMS-to-Z mechanism must not be confused with SenseOffset

`FEATURE_DIGIT_Z_COMPENSATION` is **enabled** in the current configuration, with `g_nDigitZCompensationDigits_active=true` initially. When bed compensation is active, `recalculateHeatBedZCompensation()` can add a bounded, instantaneous force-dependent Z adjustment from DMS readings. This is **not** `g_nSensiblePressureOffset` (which accumulates positive micrometres).

Thus 'no new SenseOffset during M3912' does **not** literally mean 'no force-dependent Z motion during M3912'. The start-line design must specify whether instantaneous Digit-Z protection stays active to counter force-induced nozzle flex, or whether strict pressure-only feed/E control also suspends Digit-Z. **Do not silently disable it without measuring the mechanical consequences.**

## Sources

- [G-code transform and origin handling](../../Repetier/Printer.cpp#L458-L510), [origin storage](../../Repetier/Printer.cpp#L639-L665)
- [Nominal step coordinate](../../Repetier/Printer.h#L1071-L1073), [layer recognition / g_min g_max](../../Repetier/motion.cpp#L173-L220)
- [Bed profile, fade and ECMP](../../Repetier/RF.cpp#L3192-L3302), [matrix extrema](../../Repetier/RF.cpp#L5145-L5167)
- [Actual E-step multiplier](../../Repetier/motion.cpp#L122-L135), [ECMP stepping](../../Repetier/motion.cpp#L1468-L1494)
- [Logical E reset](../../Repetier/Printer.h#L1035-L1042), [M3912 entry and exit](../../Repetier/RF.cpp#L9103-L9275)
- [User speed/flow overrides](../../Repetier/Commands.cpp#L210-L227), [runtime XY timing](../../Repetier/motion.cpp#L1645-L1654)
- [Digit-Z flag](../../Repetier/RF.cpp#L200-L210), [its additive correction](../../Repetier/RF.cpp#L3262-L3294)
