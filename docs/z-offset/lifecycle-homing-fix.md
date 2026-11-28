# Z homing as safe first-layer lifecycle boundary

**Status:** code committed on `ai-firmware-review`. Static inspections done; no AVR firmware build or physical-printer test. All three machine models share the modified code. This intentionally does **not** claim to fix consecutive prints without Z homing.

## Implementation

- `homeZAxis()` disables compensation and waits for its applied correction to unwind. It resets `Printer::layers` and restores the 0.33 and 10 mm limits at the current Z steps/mm. The SenseOffset reset clears the live value, threshold and sample history. `ZOffset` derives effective steps from the stored manual offset. The float conversions run before the motor reference update under interrupt protection. [Homing code](../../Repetier/Homing.cpp) · [Original homing fix](https://github.com/Nibbels/Repetier-Firmware/commit/ba93708d41c84fcce16872720b6b954da70ea157)
- [`handleStrainGaugeFeatures()`](../../Repetier/PressureFeatures.cpp), [733833a](https://github.com/Nibbels/Repetier-Firmware/commit/733833a7c062b461c9fbb9de8a72ea202a65c812): whenever bed CMP is disabled, drop any remaining dynamic SenseOffset, regardless of whether `M3909 P0` already deactivated it. While CMP is active, `M3909 P0` **retains** the already-applied positive correction for safety. Clear stale sample sum/count and previous-pressure reading whenever pressure regulation is inactive.

These two changes preserve the Mod Menu transfer: any portion of SenseOffset deliberately moved into the saved manual Z-offset using Z-up/down remains in EEPROM and survives homing. The untransferred RAM remainder is discarded.

## What is NOT fixed

- A second print started without `G28 Z` or full `G28` can inherit old learned `g_min`/`g_max` during priming. `M3000` clears SenseOffset. `M3912` clears all layer markers and uses its phase flag to select the DMS brake, with fade geometry inherited until the next layer is learned.
- X-only/Y-only homing must not reset Z state.
- `M3902 Z0` moves a manual offset into RAM matrix with separate EEPROM persistence; not addressed.
- `stopPrint()` marks cancellation. The queue-depth printing flag tracks activity.

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

Review Z-compensation current and target motor steps before physical tests. Keep positive clearance corrections applied while CMP is active near the bed. Physical printer testing is pending.
