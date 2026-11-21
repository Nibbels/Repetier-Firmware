# Z homing as safe first-layer lifecycle boundary

**Status:** code committed on `ai-firmware-review`. Static inspections done; no AVR firmware build or physical-printer test. All three machine models share the modified code. This intentionally does **not** claim to fix consecutive prints without Z homing.

## Implementation

- [`Printer.cpp::homeZAxis()`](../../Repetier/Printer.cpp#L1556-L1657), [ba93708](https://github.com/Nibbels/Repetier-Firmware/commit/ba93708d41c84fcce16872720b6b954da70ea157): after `setHomed(-1,-1,false)` has disabled CMP and waited for its correction to return to zero, clear `queuePositionZLayerGuessNew`, `Current`, `Last`, reset `g_minZCompensationSteps` / `g_maxZCompensationSteps` to 0.33 / 10 mm converted with actual motor steps/mm, update `g_diffZCompensationSteps`, clear SenseOffset's RAM remainder, active digits and marker, and reconstruct `g_staticZSteps` from **preserved manual `Printer::ZOffset`**. No EEPROM write; scanned bed matrix is untouched. Printing-mode guard protects milling configurations. Floating-point conversions take place *outside* the interrupt-protected block.
- [`RF.cpp::handleStrainGaugeFeatures()`](../../Repetier/RF.cpp#L5623-L5710), [733833a](https://github.com/Nibbels/Repetier-Firmware/commit/733833a7c062b461c9fbb9de8a72ea202a65c812): whenever bed CMP is disabled, drop any remaining dynamic SenseOffset, regardless of whether `M3909 P0` already deactivated it. While CMP is active, `M3909 P0` **retains** the already-applied positive correction for safety. Clear stale sample sum/count and previous-pressure reading whenever pressure regulation is inactive.

These two changes preserve the Mod Menu transfer: any portion of SenseOffset deliberately moved into the saved manual Z-offset using Z-up/down remains in EEPROM and survives homing. The untransferred RAM remainder is discarded.

## What is NOT fixed

- A second print started without `G28 Z` or full `G28` can inherit old learned `g_min`/`g_max`. `M3000` clears SenseOffset but not layer bounds; `M3912` only clears some markers **after** priming. Resetting min/max during an already-active print or only after M3912 would be risky. A future explicit new-print boundary may be needed.
- X-only/Y-only homing must not reset Z state.
- `M3902 Z0` moves a manual offset into RAM matrix with separate EEPROM persistence; not addressed.
- `stopPrint()` and SD/host print flags are not reliable new-print boundaries.

## Regression tests to run on hardware

| Scenario | Expected |
| --- | --- |
| Print A first layer 0.40, then `G28 Z`, `M3001`, `M3912`, Print B first layer 0.20 | Before M3912: min=0.33 mm, max=10 mm, dynamic=0; priming uses Digit Flow; first model layer learns 0.20 |
| Dynamic +200 µm, manual 0; perform 20 complete Mod Menu Z-DOWN/UP pairs, then `G28 Z` | Manual +100 µm retained in EEPROM; dynamic remainder +100 µm reset; no EEPROM ZOffset lost |
| `M3909 P0` during active first layer | Already applied clearance retained; new SenseOffset adjustments stop; old samples cleared |
| `M3909 P0`, then `M3000` | CMP disabled, remaining dynamic cleared even though digits already 0 |
| `M3000` without `G28 Z`, then `M3001` | Dynamic cleared, but learned min/max **remain**: not a new-print reset |
| `G28 X` or `G28 Y` | No Z layer-boundary reset |
| Print A, Print B **without** Z home or CMP disable | **Still an open state-leak scenario** |
| Build with milling enabled | Z home in printing mode resets first-layer state; milling mode does not alter it |

Review Z-compensation current/target motor steps before attempting physical tests; do not remove a positive clearance correction while CMP is active near the bed. AVR compilation and actual printer operation have **not** been validated.
