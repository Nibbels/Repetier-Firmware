# Z-offset and compensation subsystem — engineering map

> **New-chat entry point.** Reviewed against `ai-firmware-review` on 2026-11-21. Read this together with [the first-layer handover](../first-layer-control/README.md). No firmware changes or live-printer tests were made as part of this review. The shared core applies to RF1000, RF2000 and RF2000v2.

## What the mechanics do

The print gap is affected by: (1) the nominal commanded Z, (2) the scanned heat-bed shape, (3) a persistent operator/menu Z-offset, (4) a positive, capped SenseOffset correction when first-layer force is excessive, (5) optional extra force-dependent Digit-Z compensation, and (6) extruder/tool positioning. The machine also **physically drifts thermally** as the bed, nozzle and structure expand; not every physical change is estimated or automatically corrected in software.

**SenseOffset is not a replacement for the scan or for calibration.** An excessively close nozzle can increase extrusion force because the gap provides less outlet space for polymer. A DMS strain gauge measures force, not melt pressure. Pressure can also rise due to a clog, speed, material, temperature, filament/cable preload or a dirty nozzle.

### The software Z stack (conceptual, not a complete machine coordinate transformation)

```text
Commanded Z (the print's planned geometric height)
  + scanned-bed correction B(x,y), progressively faded to its baseline O
  + g_staticZSteps = (Printer::ZOffset + g_nSensiblePressureOffset)*Z_steps_per_mm/1000
  + optional force-sensitive Digit-Z correction
  + other guarded mechanical/tool effects, Z endstop limits
  → compensatedPositionTargetStepsZ → physical Z motor activity
```

The **nominal Z** is also used to decide the fade region. The source does *not* simply sum all these terms at all heights: below `g_min` it follows the full matrix, above `g_max` it retains the matrix baseline, and there are endstop checks. Tool-specific Z offsets have their own coordinate path; see [offset ledger](offset-ledger.md).

**Critical dual use:** `g_minZCompensationSteps` is both the full-profile fade boundary **and** the SenseOffset eligibility height. It also determines whether high-force priming runs Digit Flow Compensation (`currentZ > g_min`), the opposite of the SenseOffset low-Z gate (`currentZ <= g_min`). A stale `g_min` can change which controller reacts.

## Files to read

1. [Offset ledger and actual formulas](offset-ledger.md) — persistent vs live and what contributes.
2. [Lifecycle and possible Z jumps](lifecycle-and-findings.md) — boot, Z homing versus stepper disable, consecutive prints, M3006, M3909, menu, matrix save; prioritized source-backed findings.
3. [Full heat-bed scan, point Z-offset scan and storage](scans-and-persistence.md) — calibration procedures, what is committed and when.
4. [Thermal drift and strain-gauge uncertainty](thermal-and-force.md) — historical coefficients, approximate physical behavior, error sources.
5. [First-layer fade-out and ECMP](../first-layer-control/z-compensation-and-ecmp.md), [SenseOffset](../first-layer-control/sense-offset.md), [adaptive M3912 priming](../first-layer-control/automatic-start-line.md) — allied mechanisms.

## Caution for firmware fixes

**Do not reset a positive SenseOffset abruptly in mid-print:** that can move the nozzle closer and damage a part or bed. Design a **new-print / old-print boundary** and appropriate safe Z state first; separate logical 'controller enabled' from retaining/removing already-applied physical offset. Test with real G-code sequences. Do not treat a perceived random Z shift as proven to have a single cause.

See [lifecycle-and-findings.md](lifecycle-and-findings.md) for the concrete issues discovered.
