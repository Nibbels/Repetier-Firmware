# First-layer pressure, scanned-bed correction and priming

> **New-chat handover.** Based on `ai-firmware-review` in November 2026. The mechanisms described here share firmware logic on RF1000, RF2000 and RF2000v2. Source is authoritative; unproven defects are labelled hypotheses.

**Related deep dive:** [Z-offset and compensation subsystem](../z-offset/README.md) — manual+dynamic offsets, matrix storage, boot/print lifecycle, scanning and thermal effects.

**Coordinate interpretation:** [nominal G-code Z → planned steps → bed-corrected motor steps, plus E logical timeline](coordinate-frames-and-extrusion.md). In particular, learned `g_min`/`g_max` are nominal planner Z steps, not physically compensated machine positions.

## Problem being solved

A heat-bed scan supplies the bed's geometric shape. It cannot fully predict real nozzle-to-bed clearance at printing temperature: thermal expansion, offsets, nozzle differences and mechanical variability change the true gap. A very narrow first-layer gap creates strong extrusion resistance. DMS strain gauges infer **force** associated with extrusion; this is *not* a direct measurement of melt pressure and cannot uniquely distinguish insufficient gap from a clog, wrong temperature, material viscosity or excessive feed.

## Three different controls — do not conflate them

| System | Input | What it does | Where |
| --- | --- | --- | --- |
| [**SenseOffset**](sense-offset.md), `M3909` | DMS force above limit | Incrementally adds a **positive Z clearance offset**, with a maximum; never drives closer in response to low pressure | Eligible near-bed first-layer Z only, with heat-bed compensation on |
| [**Bed-profile fade / ECMP**](z-compensation-and-ecmp.md) | Scanned XY matrix, detected layer height | Follow measured bed shape on first layer; gradually fade the variable correction to a common offset; compensate E material for layer-thickness changes | First layer through fade-out height |
| [**Adaptive M3912 start line**](automatic-start-line.md) | DMS force during priming | Temporarily reduces **travel speed and E extrusion** | Priming at nominal Z=0.35 mm |

Digit-Z adds a force-dependent Z adjustment when enabled in the DMS menu. Emergency pause and stop use their own force limits.

### Intended print sequence

1. Home, then enable scanned-bed compensation with `M3001`.
2. `M3912` primes at nominal Z=0.35 mm using short segments. The prime follows the scanned bed and uses the DMS speed and E brake. The active priming flag suspends layer learning and SenseOffset.
3. Enable SenseOffset with `M3909` after the start line. For a manual line, keep SenseOffset AutoStart off so `M3001` only enables bed compensation.
4. The planner learns the first model layer when positive extrusion confirms a new Z or when the first XY extrusion begins at the current Z. The layer height sets `g_minZCompensationSteps`. `g_maxZCompensationSteps` sets the fade-out span.
5. Full scanned-bed tracking and the bounded SenseOffset controller operate near that real layer. Higher up, bed-profile variation fades away while corresponding E-material correction helps maintain suitable deposited volume.

### Important units/state

| Name | Meaning |
| --- | --- |
| `Printer::queuePositionZLayerGuessNew` | Candidate Z layer after a Z move, pending positive extrusion |
| `queuePositionZLayerCurrent` / `Last` | Accepted current/previous model layers |
| `g_minZCompensationSteps` | Full bed-profile limit **also used to gate SenseOffset** |
| `g_maxZCompensationSteps` | End of fade-out of spatial bed shape |
| `g_offsetZCompensationSteps` | Matrix-derived baseline |
| `g_staticZSteps` | Manual menu Z offset + SenseOffset (converted from micrometres to steps) |
| `Printer::compensatedPositionOverPercE` | Geometric extrusion correction from changing layer thickness |
| `dynamicFeedrateFactor` / `dynamicExtrusionFactor` | Pressure-dependent feed and extrusion multipliers for Digit Flow Compensation |

Positions named *Steps* are motor steps, SenseOffset and manual Z offsets are in **um**, DMS forces in **digits**.

### Priming and consecutive prints

`M3912` identifies the generated prime through its phase flag. The ordinary layer detector can learn a model layer at 0.352 mm or at the priming height of 0.35 mm. Z homing resets the learned geometry. If it is skipped, the previous print's `g_min/g_max` can still affect the bed-profile fade during priming. [M3912 implementation](m3912-state-boundary.md)

Continue with [SenseOffset](sense-offset.md), [fade-out / ECMP](z-compensation-and-ecmp.md), [M3912](automatic-start-line.md), [test scenarios](test-scenarios.md), [history](history.md).
