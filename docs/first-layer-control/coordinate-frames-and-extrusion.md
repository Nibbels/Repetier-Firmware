# Coordinate frames, bed-profile fade-out and the extrusion timeline

> Based on the shared RF1000, RF2000 and RF2000v2 source on `ai-firmware-review`. Host tests replayed the planner arithmetic with binary32 floats and simulated motors. AVR builds passed for all three models.

## Z: four different layers of meaning

1. **Theoretical slicer G-code**: `G1 Z0.20` requests a nominal layer height in the G-code's current work origin, G90/G91 mode and units. This number is not necessarily physical Z motor steps.
2. **Uncompensated planned firmware position.** `Printer::queueGCodeCoordinates()` converts units and subtracts the work-origin offset in absolute mode. The planner rounds the float target with `prepareDestinationSteps(Z_AXIS)` and records the resulting integer in `queuedPositionSteps[Z_AXIS]`. `currentSteps[Z_AXIS]` tracks execution. The learned `g_min` and `g_max` use the nominal planned steps.
3. **Compensation/Z motor target**: `recalculateHeatBedZCompensation()` uses the *executing nominal* `currentSteps[Z_AXIS]` to select the height-dependent shape compensation and adds `g_staticZSteps` (menu Z + accumulated SenseOffset). A separate optional **Digit-Z** force correction and endstop rules may also apply. Target is `compensatedPositionTargetStepsZ`; actual motor correction `compensatedPositionCurrentStepsZ` can lag behind the target.
4. **Real nozzle-to-bed clearance and printed geometry** combine nominal Z motion, actual compensation motor steps, direct Z moves, physical thermal drift and the real bed surface.

The layer detector and fade calculation use nominal Z. Manual direct moves can change nozzle clearance independently of that value.

## Why segment rounding cannot accumulate in XYZ

For a fixed calibration, define `P(i)` as the rounded integer endpoint of segment `i`. The planner queues `P(i) - P(i-1)`. Adding those deltas cancels the intermediate endpoints and leaves `P(last) - P(start)`. Returning to the same represented target therefore returns to the same nominal step, regardless of segment size and direction. Homing and print cancellation set the queued endpoint and executing position together. [Planner](../../Repetier/motion.cpp#L117-L168) · [Coordinate resets](../../Repetier/Printer.h#L1015-L1041)

The previous planner repeatedly added step-sized distances to an XYZ float. The float could drift relative to the integer pulse total. A half-step target could also shift on a later move that left the target unchanged.

| Host replay | Previous planner | Integer endpoints |
| --- | --- | --- |
| Z0 to Z200 in 0.001 mm targets, then one move to Z0, at 1,280 steps/mm | 459 steps from zero | 0 steps from zero |
| Z at 2.5 steps, followed by Y moves | Z shifted from step 3 to step 2 | Z stayed at step 3 |

Relative moves keep the rounding remainder from each float addition and fold it into later updates. Absolute axis targets clear that axis's remainder. The XYZ planner still rounds the float target once per endpoint. E uses the difference between both parts of its logical target before applying flow. [Coordinate update](../../Repetier/Printer.cpp#L455-L497) · [Planner](../../Repetier/motion.cpp#L120-L168)

| Relative G-code replay | Previous accumulation | With the remainder |
| --- | --- | --- |
| 200,000 × Z0.0005, then Z−100 at 1,280 steps/mm | +0.2253 mm in the target, 288 steps from zero | About +0.00000475 mm in the target, 0 steps from zero |
| 100,000 × E0.008 at 280 steps/mm and 100% flow | 223,903 base steps | 224,000 base steps |

The Z residual reflects the precision of each parsed binary32 input. Actual motor position also depends on the configured limits, direct moves, compensation and the motor following its pulses.

WobbleFix adds an XY correction at the step endpoint. Omitted axes and relative moves use the retained target in millimetres. E-only moves reuse the queued XY correction, and homing or setting an explicit machine X/Y target clears the correction on that axis. The software limit constrains the retained target and the corrected endpoint independently. [Conversion to steps](../../Repetier/Printer.cpp#L285)

## What `g_min` and `g_max` really mean

`PrintLine::prepareQueueMove()` accepts a prospective layer when positive extrusion follows a Z move. With no tracked layer, an XY extrusion move can also establish the first layer at the current nominal Z. The planner records `Printer::queuePositionZLayerCurrent` and computes the following boundaries.

```cpp
g_minZCompensationSteps = Printer::queuePositionZLayerCurrent;
g_maxZCompensationSteps = g_minZCompensationSteps
    + (g_offsetZCompensationSteps - g_ZCompensationMax) * 20;
```

- **`g_min` = actual first model layer's nominal planned Z** (not the distance from the *physically uneven* bed at every XY point). Full bed-shape following extends through that nominal layer.
- **`g_max` = the nominal Z at which XY-dependent bed-shape variation has completely faded**. It depends on matrix relief, not a fixed number of model layers.
- These are **boundaries in the *input* nominal Z frame**, not Z corrections themselves or already-compensated physical positions. Their values are stored in motor **steps**, not mm or raw G-code strings.
- The default 0.33 mm/10 mm are fallbacks; the first model layer dynamically replaces the lower limit and the formula sets the upper.
- The `M3912` phase flag suspends learning while generated prime moves are queued. The model can establish a first layer at the same height as the prime.

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

For absolute-E G-code, the slicer's **logical E timeline** gives a requested change in filament length. The planner applies the flow multipliers and computes base E steps. It carries the fractional step in `[-0.5, 0.5)` and initializes each segment's pulse accumulator, including reused queue slots. Motion direction and distance use the resulting integer step count. ECMP and advance can add their own pulses during execution. [Extrusion quantization](../../Repetier/motion.cpp#L120-L161)

`G92 E` sets both the destination and queued logical E coordinates and clears their millimetre remainders. The fractional motor-step carry survives, so setting a fractional E origin followed by a move to the same E value produces zero new base pulses. A push followed by an equal undo at an exact half-step also returns to the same count with constant flow. [G92 dispatch](../../Repetier/Commands.cpp#L394)

### Arithmetic test: restore the logical timeline

Suppose the slicer is at absolute `E10`, then calls `M3912`, whose internal priming commands end near logical `E50`. The slicer's next move is `E11`.

| End of M3912 | Next slicer E11 interpreted as | Result |
| --- | --- | --- |
| Old implementation before 5f848c9: force logical E0 | +11 E units | **Wrong if M3912 was called after E10** |
| Patched M3912: restore entry logical E10 | +1 E unit | **Correct** |
| Conventional start script: entry E0, next slicer command E1 | +1 E unit | Both reset-to-E0 and restore-entry-E0 produce the correct result |

At entry, M3912 drains the queue, saves the incoming logical E and its millimetre remainder, and primes on its own E0 timeline. After the line it restores logical E with its remainder, E mode and fractional step accounting. Print cancellation retains the stop handler's E0. [Save and restore](../../Repetier/RF.cpp#L9096-L9271)

In relative E mode, firmware-generated absolute E coordinates remain internal to `queueFloatCoordinates()`; restoring logical E is still harmless and preserves later G-code interpretation. Retracts cause occasional decreases in the logical E timeline, but do not invalidate this model.

## DMS speed is orthogonal to user speed override

- `Printer::feedrateMultiply` (typically M220 or menu speed%) changes the user's normal requested feedrate. The stored `Printer::feedrate` incorporates that multiplier.
- `Printer::dynamicFeedrateFactor` is a separate DMS-based value (1.0 means neutral). It is translated to `Printer::interval_mod`, a timer divisor applied at **execution time** to XY-primary queued moves; it **does not modify** `feedrateMultiply`.
- `Printer::menuExtrusionFactor` (user flow%) and `Printer::dynamicExtrusionFactor` (DMS flow) likewise remain separate; the planner multiplies them when generating physical E steps.
- `M3912` uses explicit mm/s for its internal queue moves; it does not change or save the general user speed override. Consequently, don't assume that the start-line's configured XY feedrate always applies M220 in exactly the same way as a slicer G1 F move. Pressure-derived step slowdown remains an additional mechanism.
- Returning `dynamicFeedrateFactor` to 1.0 is correct **but derived `interval_mod` should also be neutralized** after priming to avoid a stale timer-scale effect.

## Two fractional E-step accumulators also cross the priming boundary

- `Printer::extrudeMultiplyErrorSteps` is the **sub-step remainder** from mapping logical E movement (with menu flow and dynamic DMS flow multipliers) into integer motor steps during queue planning.
- `Printer::compensatedPositionCollectTinyE` is the **sub-step remainder** from adding geometric ECMP extrusion during step execution.

Both previously spilled into M3912. The patched code **saves and restores both fractional E-step remainders** after the queue drains, with interrupt protection for the ECMP remainder. Hardware validation remains necessary.

## ECMP and partial bed fade also run during the current priming line

At the freshly homed default `g_min=0.33 mm`, M3912 primes at nominal Z=0.35 mm, about **20 µm into the fade region**. `recalculateHeatBedZCompensation()` can apply that small fade and compute geometric E compensation during priming.

With the dynamic fade span determined from `(O-minB)*20`, the shape loss at the priming height is generally small. ECMP may still follow the corresponding fade slope. A physical test can quantify its effect on deposited material during priming.

[Source: ECMP percent during fade](../../Repetier/RF.cpp#L3199-L3254) · [additional E-step accumulation](../../Repetier/motion.cpp#L1468-L1492) · [fractional E rounding](../../Repetier/motion.cpp#L125-L136).

## Elastic Digit-Z correction during priming

Digit-Z is built into all three models and enabled by default through `g_nDigitZCompensationDigits_active`. During M3912, bed compensation can still include a bounded correction for elastic hotend deflection. It follows filtered force as the load rises and falls. M3912 suspends the accumulation of SenseOffset during that phase. [Digit-Z formula and timing](../z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)

## Sources

- [G-code transform and origin handling](../../Repetier/Printer.cpp#L458-L510), [origin storage](../../Repetier/Printer.cpp#L639-L665)
- [Nominal step coordinate](../../Repetier/Printer.h#L1071-L1073), [layer recognition / g_min g_max](../../Repetier/motion.cpp#L173-L220)
- [Bed profile, fade and ECMP](../../Repetier/RF.cpp#L3192-L3302), [matrix extrema](../../Repetier/RF.cpp#L5145-L5167)
- [Actual E-step multiplier](../../Repetier/motion.cpp#L122-L135), [ECMP stepping](../../Repetier/motion.cpp#L1468-L1494)
- [Logical E reset](../../Repetier/Printer.h#L1035-L1042), [M3912 entry and exit](../../Repetier/RF.cpp#L9103-L9275)
- [User speed/flow overrides](../../Repetier/Commands.cpp#L210-L227), [runtime XY timing](../../Repetier/motion.cpp#L1645-L1654)
- [Digit-Z flag](../../Repetier/RF.cpp#L200-L210), [its additive correction](../../Repetier/RF.cpp#L3262-L3294)
