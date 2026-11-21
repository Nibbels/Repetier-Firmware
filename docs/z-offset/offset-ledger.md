# Offset ledger: meaning, units, persistence and consumers

## Firmware-level offset accounting

| Term | Unit | Persistence | Writer / update | Consumer / effect |
| --- | --- | --- | --- | --- |
| `Printer::ZOffset` | µm, signed `long` | **Internal MCU EEPROM** `EPR_RF_Z_OFFSET`; restored on boot | `M3006 S<um>` / `Z<mm>`, Z-offset menu/keys, `M3902 Z0` transfer | Basis of `g_staticZSteps` |
| `g_nSensiblePressureOffset` | µm, nonnegative `short` | **RAM only**, starts at 0 on boot; **not intrinsically cleared at end of print** | Force-based controller; Z-minus key may decrease it first | Added to menu offset in `g_staticZSteps`; physical clearance *increases* |
| `g_staticZSteps` | Z motor steps | Derived RAM, not individually saved | EEPROM load, M3006, menu actions, SenseOffset, mode setup, matrix transfer | Added to print Z compensation target |
| `g_ZCompensationMatrix[x][y]` | signed Z motor steps (header XY in other units) | Working RAM; saved separately to **external I²C EEPROM** matrix sector | Heat-bed scan, Z-offset scan M3900, M3902 matrix manipulation, matrix load | Interpolated bed topography `B(x,y)` |
| `g_offsetZCompensationSteps` | steps | Derived RAM from maximum matrix value, *not* a separate user offset | `determineCompensationOffsetZ()` on matrix load/edit | Baseline `O` at and above fade end |
| `g_ZCompensationMax` | steps | Derived RAM from minimum matrix value | Same matrix-extrema routine | Sets fade span with `g_offsetZCompensationSteps` |
| `g_minZCompensationSteps` | steps | RAM: initial/fallback config; **dynamically replaced** by first-layer detector | Boot EEPROM read; `motion.cpp` model layer learning | Full bed-following limit, SenseOffset low-Z gate, Digit Flow high-Z gate |
| `g_maxZCompensationSteps` | steps | RAM: fallback config then dynamically learned | Boot EEPROM read; `motion.cpp` | End of scanned-profile fade and ECMP transition |
| `g_nDigitZCompensationDigits` | force digits (stored float) | RAM | DMS readings | Optional additional force-dependent Z target term |
| `Extruder::current->offsetMM[Z_AXIS]` | mm | Internal EEPROM per extruder | Tool configuration/menu | Extruder/tool-specific positioning, not simply interchangeable with `Printer::ZOffset` |
| `Printer::compensatedPositionTargetStepsZ` / `CurrentStepsZ` | steps | RAM/live stepper state | Z compensation target calculation / execution | Separate desired vs realized correction; can lag during motion |

### Simplified formula for printing (normal Z >= 0)

Let `z` be nominal Z in motor steps, `B(x,y)` the interpolated matrix value, `O=g_offsetZCompensationSteps`, `a=g_minZCompensationSteps` and `b=g_maxZCompensationSteps`.

```text
BedPart =
  B(x,y)                                    if z <= a
  O + (B(x,y)-O)*(b-z)/(b-a)                if a < z < b
  O                                         if z >= b

g_staticZSteps =
  trunc((Printer::ZOffset + g_nSensiblePressureOffset) * Z_steps_per_mm / 1000)

CompensationTarget =
  BedPart + g_staticZSteps
  + optional Digit-Z correction
  + application-specific bounds/special cases
```

This describes **the compensation target** rather than absolute physical nozzle coordinates. Endstop constraints, negative Z, direct moves, queue moves, tool offsets, and disabled compensation matter. It is an aid for debugging, not a replacement for reading `recalculateHeatBedZCompensation()`.

**Sign:** positive menu Z offset / positive SenseOffset raises nozzle-to-bed clearance by moving the bed away in printing convention. An edit to a matrix shifts the baseline and may change clearance without changing the menu Z-offset.

## Operator's intended live transfer

In the Mod Menu (page 2), **alternating the Z/bed DOWN and UP buttons transfers 5 µm per completed pair from RAM SenseOffset to EEPROM-backed `Printer::ZOffset`** while maintaining approximately the same total correction. This was explicitly documented in the original January 2017 changelog, and is a key workflow for persistent live calibration. See [exact example and edge cases](history-and-workflows.md). Ordinary menu editing and `M3006` both retain the combined correction after [M3006 fix b4ae1dc](https://github.com/Nibbels/Repetier-Firmware/commit/b4ae1dc53a184c7a866db987f4419413912a58d5), but `M3006` assigns a manual offset directly rather than performing the deliberate transfer. Do not remove this feature during lifecycle fixes.

## Examples of 'same look, different stored state'

- A +50 µm manual offset plus +30 µm SenseOffset produces a nominal **+80 µm derived static correction**.
- `M3902 Z0` can move the +50 µm *manual portion* into the active RAM matrix and set the saved menu offset to zero. If the matrix is then successfully saved in external EEPROM, the effective baseline is approximately unchanged (subject to step rounding and scan limits).
- If the menu offset is zeroed/persisted but the RAM matrix **is not saved**, reboot restores zero manual offset and the old external matrix. The nozzle gap is now missing the transferred +50 µm. This is a **real non-atomic persistence mismatch**, not random mechanical expansion.
- Dynamic SenseOffset +30 µm is not automatically baked into `M3902 Z0`; the transfer uses `Printer::ZOffset` only.

## Location and evidence

- [Global definitions](../../Repetier/RF.cpp#L69-L75), [SenseOffset and static offset globals](../../Repetier/RF.cpp#L153-L199)
- [EEPROM boot load of manual offset and min/max](../../Repetier/Eeprom.cpp#L605-L624)
- [Menu actions](../../Repetier/ui.cpp#L2821-L2878), [Z-offset configuration menu](../../Repetier/ui.cpp#L3166-L3176)
- [M3006](../../Repetier/RF.cpp#L6587-L6635), [M3902 Z0](../../Repetier/RF.cpp#L8855-L8952)
- [Determine matrix minimum and maximum](../../Repetier/RF.cpp#L5145-L5167)
- [Actual Z-compensation calculation](../../Repetier/RF.cpp#L3192-L3302)
- [First-layer learning of fade boundaries](../../Repetier/motion.cpp#L173-L220)

Related: [lifecycle and findings](lifecycle-and-findings.md).
