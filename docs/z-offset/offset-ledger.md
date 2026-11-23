# Offset ledger: meaning, units, persistence and consumers

## Firmware-level offset accounting

| Term | Unit | Persistence | Writer / update | Consumer / effect |
| --- | --- | --- | --- | --- |
| `Printer::zOffset.manualMicrometers()` | µm, signed `int32_t` | **Internal MCU EEPROM** `EPR_RF_Z_OFFSET` | EEPROM reload, `M3006 S<um>` / `Z<mm>`, Z-offset menu and keys, `M3902 Z0` | Manual portion of the effective offset |
| `Printer::zOffset.senseMicrometers()` | µm, nonnegative `int16_t` | **RAM only**, starts at 0 on boot | Force-based learning, Mod Menu Bed UP, Z homing and compensation-off cleanup | Adds clearance to the manual offset |
| `Printer::zOffset.effectiveSteps()` | Z motor steps | Derived RAM | `ZOffset` computes it from both inputs and the current Z steps/mm | Added to the print Z compensation target |
| `g_ZCompensationMatrix[x][y]` | signed Z motor steps (header XY in other units) | Working RAM; saved separately to **external I²C EEPROM** matrix sector | Heat-bed scan, Z-offset scan M3900, M3902 matrix manipulation, matrix load | Interpolated bed topography `B(x,y)` |
| `g_offsetZCompensationSteps` | steps | Derived RAM from maximum matrix value, *not* a separate user offset | `determineCompensationOffsetZ()` on matrix load/edit | Baseline `O` at and above fade end |
| `g_ZCompensationMax` | steps | Derived RAM from minimum matrix value | Same matrix-extrema routine | Sets fade span with `g_offsetZCompensationSteps` |
| `Printer::layers.fadeStartSteps()` | nominal Z steps | RAM, fallback then learned from the first model layer | `PrintLayers` on planning, homing, EEPROM load and microstep changes | Full bed-following limit, SenseOffset low-Z gate, Digit Flow high-Z gate |
| `Printer::layers.fadeEndSteps()` | nominal Z steps | RAM, fallback then derived from the first model layer and matrix relief | `PrintLayers` on planning, homing, EEPROM load and microstep changes | End of scanned-profile fade and ECMP transition |
| `Printer::digitCompensation.filteredDigits()` | signed force digits | RAM | Foreground DMS filter | Optional additional force-dependent Z target term |
| `ZMotion` correction target and applied correction | Z motor steps | RAM/live stepper state | Bed and DMS calculation publishes the target. `ZMotion` updates the applied correction. | Applied correction can lag behind the target during motion |

### Simplified formula for printing (normal Z >= 0)

Let `z` be the executing nominal Z in motor steps, `B(x,y)` the interpolated matrix value, and `O=g_offsetZCompensationSteps`. The bounds `a` and `b` are `layers.fadeStartSteps()` and `layers.fadeEndSteps()`.

```text
BedPart =
  B(x,y)                                    if z <= a
  O + (B(x,y)-O)*(b-z)/(b-a)                if a < z < b
  O                                         if z >= b

EffectiveOffsetSteps =
  trunc((manualMicrometers + senseMicrometers) * Z_steps_per_mm / 1000)

CompensationTarget =
  BedPart + EffectiveOffsetSteps
  + optional Digit-Z correction
  + application-specific bounds/special cases
```

This formula describes the compensation target. Endstop constraints and the executing movement determine the resulting motor position. The implementation handles negative Z, zero-height clearance, and disabled compensation. [Bed correction](../../Repetier/BedCompensation.h) · [Target publication](../../Repetier/RF.cpp#L1944)

**Sign:** positive menu Z offset / positive SenseOffset raises nozzle-to-bed clearance by moving the bed away in printing convention. An edit to a matrix shifts the baseline and may change clearance without changing the menu Z-offset.

## Operator's intended live transfer

In the Mod Menu (page 2), **alternating the Z/bed DOWN and UP buttons transfers 5 µm per completed pair from RAM SenseOffset to the EEPROM-backed manual offset** while maintaining approximately the same total correction. The January 2017 changelog documents this workflow for persistent live calibration. Ordinary menu editing and `M3006` set the manual value and preserve the live portion. [Button examples and edge cases](history-and-workflows.md)

## Examples of 'same look, different stored state'

- A +50 µm manual offset plus +30 µm SenseOffset produces a nominal **+80 µm derived static correction**.
- `M3902 Z0` can move the +50 µm *manual portion* into the active RAM matrix and set the saved menu offset to zero. If the matrix is then successfully saved in external EEPROM, the effective baseline is approximately unchanged (subject to step rounding and scan limits).
- If the menu offset is zeroed/persisted but the RAM matrix **is not saved**, reboot restores zero manual offset and the old external matrix. The nozzle gap is now missing the transferred +50 µm. This is a **real non-atomic persistence mismatch**, not random mechanical expansion.
- For `M3902 Z0`, the matrix shift uses the manual portion and the +30 µm remains live.

## EEPROM reload and steps/mm

`M501` and `M206` reload the saved manual value and use the current live SenseOffset to derive the combined steps. With +50 µm saved and +30 µm live at 1,280 steps/mm, a reload now keeps 102 steps. The previous calculation dropped to 64 steps while retaining +30 µm in RAM. Firmware startup begins with zero live offset. Changes to Z steps/mm, including the microstep menu and factory restore, recompute from the two micrometer values. The step timer still consumes the published compensation target. [Offset owner](../../Repetier/ZOffset.h) · [EEPROM reload](../../Repetier/Eeprom.cpp#L404) · [Compensation target](../../Repetier/RF.cpp#L2045)

Run the host regression from the repository root.

```sh
c++ -std=c++11 -Wall -Wextra -Werror tests/z-offset.cpp -o /tmp/rfx-z-offset-test
/tmp/rfx-z-offset-test
```

## Code paths

- [Mod Menu buttons](../../Repetier/ui.cpp#L2335) and [M3006](../../Repetier/RF.cpp#L3895) edit the manual and live values through the owner.
- [M3902 Z0](../../Repetier/RF.cpp#L5539) transfers the manual offset into the matrix. [Matrix extrema](../../Repetier/RF.cpp#L2807) determine the baseline for the interpolated correction.
- [Layer detection](../../Repetier/motion.cpp#L173) sets the learned fade boundaries.

Related: [lifecycle and findings](lifecycle-and-findings.md).
