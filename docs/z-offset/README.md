# Z-offset and compensation subsystem — engineering map

> **New-chat entry point.** Reviewed against `ai-firmware-review` on 2026-11-21. Read this together with [the first-layer handover](../first-layer-control/README.md). No firmware changes or live-printer tests were made as part of this review. The shared core applies to RF1000, RF2000 and RF2000v2.

## What the mechanics do

The print gap depends on nominal Z, the scanned heat-bed shape, the saved manual Z offset, SenseOffset and Digit-Z. The machine also drifts as the bed, nozzle and structure heat up.

**SenseOffset is not a replacement for the scan or for calibration.** An excessively close nozzle can increase extrusion force because the gap provides less outlet space for polymer. A DMS strain gauge measures force, not melt pressure. Pressure can also rise due to a clog, speed, material, temperature, filament/cable preload or a dirty nozzle.

### The software Z stack (conceptual, not a complete machine coordinate transformation)

```text
Commanded Z (the print's planned geometric height)
  + scanned-bed correction B(x,y), progressively faded to its baseline O
  + Printer::zOffset.effectiveSteps() = trunc((manual µm + SenseOffset µm)*Z_steps_per_mm/1000)
  + optional force-sensitive Digit-Z correction
  + other mechanical effects and Z endstop limits
  → compensatedPositionTargetStepsZ → physical Z motor activity
```

The **nominal Z** selects the fade region. Below `g_min` the correction follows the full matrix, and above `g_max` it uses the matrix baseline. Z endstops constrain the result. [Offset ledger](offset-ledger.md)

`g_minZCompensationSteps` is both the full-profile fade boundary and the SenseOffset eligibility height. Ordinary Digit Flow is eligible at `currentZ > g_min`; SenseOffset is eligible at `currentZ <= g_min`. The `M3912` phase flag enables Digit Flow and suspends SenseOffset during generated priming independently of these limits. `homeZAxis()` resets the learned limits.

## Files to read

1. [Offset ledger and actual formulas](offset-ledger.md) — persistent vs live and what contributes.
2. [**Mod Menu live transfer: SenseOffset → persistent Z-offset**](history-and-workflows.md) — Z-up/down alternation, 5 µm steps and EEPROM behavior.
3. [Lifecycle and possible Z jumps](lifecycle-and-findings.md) — boot, Z homing versus stepper disable, consecutive prints, M3006, M3909, menu, matrix save; prioritized source-backed findings.
4. [Full heat-bed scan, point Z-offset scan and storage](scans-and-persistence.md) — calibration procedures, what is committed and when.
5. [Thermal drift and strain-gauge uncertainty](thermal-and-force.md) — historical coefficients, approximate physical behavior, error sources.
6. [First-layer fade-out and ECMP](../first-layer-control/z-compensation-and-ecmp.md), [SenseOffset](../first-layer-control/sense-offset.md), [adaptive M3912 priming](../first-layer-control/automatic-start-line.md) — allied mechanisms.

## Caution for firmware fixes

**Do not reset a positive SenseOffset abruptly in mid-print:** that can move the nozzle closer and damage a part or bed. Design a **new-print / old-print boundary** and appropriate safe Z state first; separate logical 'controller enabled' from retaining/removing already-applied physical offset. Test with real G-code sequences. Do not treat a perceived random Z shift as proven to have a single cause.

See [lifecycle-and-findings.md](lifecycle-and-findings.md) and [the homing lifecycle patch](lifecycle-homing-fix.md) for the fixes and remaining no-homing risk.
