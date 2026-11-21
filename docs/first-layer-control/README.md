# First-layer pressure, scanned-bed correction and priming

> **New-chat handover.** Based on `ai-firmware-review` in November 2026. The mechanisms described here share firmware logic on RF1000, RF2000 and RF2000v2. Source is authoritative; unproven defects are labelled hypotheses.

## Problem being solved

A heat-bed scan supplies the bed's geometric shape. It cannot fully predict real nozzle-to-bed clearance at printing temperature: thermal expansion, offsets, nozzle differences and mechanical variability change the true gap. A very narrow first-layer gap creates strong extrusion resistance. DMS strain gauges infer **force** associated with extrusion; this is *not* a direct measurement of melt pressure and cannot uniquely distinguish insufficient gap from a clog, wrong temperature, material viscosity or excessive feed.

## Three different controls — do not conflate them

| System | Input | What it does | Where |
| --- | --- | --- | --- |
| [**SenseOffset**](sense-offset.md), `M3909` | DMS force above limit | Incrementally adds a **positive Z clearance offset**, with a maximum; never drives closer in response to low pressure | Eligible near-bed first-layer Z only, with heat-bed compensation on |
| [**Bed-profile fade / ECMP**](z-compensation-and-ecmp.md) | Scanned XY matrix, detected layer height | Follow measured bed shape on first layer; gradually fade the variable correction to a common offset; compensate E material for layer-thickness changes | First layer through fade-out height |
| [**Adaptive M3912 start line**](automatic-start-line.md) | DMS force during priming | Temporarily reduces **travel speed and E extrusion**, not just one or the other | Special Z=0.352 mm priming line before the actual model |

Optional `FEATURE_DIGIT_Z_COMPENSATION` adds another force-dependent Z adjustment; emergency stop/pause is separate again.

### Intended print sequence

1. Home, load scanned bed data, enable Z compensation via `M3001`. SenseOffset may auto-start as an independently queued task based on EEPROM preference.
2. `M3912` primes at Z=0.352 mm, using short queued line segments so force can influence subsequent extrusion and speed. The priming line **still follows the scanned bed**.
3. Prime is deliberately **not** the first real printed layer. High priming force should slow/reduce extrusion, not activate SenseOffset's height correction.
4. When positive extrusion follows a real Z change, the shared planner identifies the model's first layer. Its height becomes `g_minZCompensationSteps`; `g_maxZCompensationSteps` sets the fade-out span.
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

### Current research question

The numeric Z=0.352 mm exclusion protects dynamic first-layer learning, but current `M3912` and Z-axis reset paths clear only some layer markers, **not** learned `g_min` / `g_max`. A previous first layer higher than 0.352 mm could leave the next prime in SenseOffset's height range and outside the Digit Flow response range. This is a **static-code hypothesis pending test**, not a verified field failure. A fork has a model-specific **after-priming** reset; do not port it uncritically.

Continue with [SenseOffset](sense-offset.md), [fade-out / ECMP](z-compensation-and-ecmp.md), [M3912](automatic-start-line.md), [test scenarios](test-scenarios.md), [history](history.md).
