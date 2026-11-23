# M3912: implemented pressure-braked priming transaction

The [priming state fix](https://github.com/Nibbels/Repetier-Firmware/commit/5f848c9a5cd0347c6d45fd9d3f13afd7344cb775) and [atomic timer cleanup](https://github.com/Nibbels/Repetier-Firmware/commit/b793cd8203e7ed71154728412a03d4e9f4d7fb67) are on `ai-firmware-review`. AVR builds and planner checks passed with the height exclusion removed. Physical printer testing is pending.

## Intent: DMS-controlled braking, not insufficient motor power

The operator's **chainsaw-under-load analogy** describes the audible effect: while a chainsaw slows under wood load, here the printer **deliberately applies a simulated, gradually increasing brake** when DMS detects high extrusion force. The motors are not necessarily at their power limit. The firmware primarily slows XY/filament throughput to relieve resistance, and additionally reduces extrusion amount. This is the original intended pressure-response mode for `M3912`; it is fundamentally different from changing permanent or dynamic Z clearance.

Default thresholds are **1500–4500 DMS digits** (overridable by `M3912 S/P`), with speed intensity −90% and extrusion intensity −75%. At high force the target speed factor reaches **0.10** and the E factor **0.25**. The speed factor approaches its target by small 0.01 steps per update; planned E segments use the instantaneous E factor. These are not direct pressure measurements.

**Preserve the user's M220 speed override** (`Printer::feedrateMultiply`) and M221 flow multiplier (`Printer::menuExtrusionFactor`). `Printer::digitCompensation` supplies `intervalScale()` to live XY-primary step timing and `extrusionFactor()` to E planning.

## Generated priming phase

`M3912` primes at nominal Z=0.35 mm. The temporary `g_startLineActive` flag sets the policy for generated priming.
- During priming, enable DMS feed/E braking **regardless of stale `g_min`** from earlier prints.
- Suppress **new SenseOffset increments**, while preserving the already applied positive correction and the EEPROM-backed manual Z-offset.
- Keep priming movement out of the first-model-layer detector, and clear its pressure sample history.
- After the prime queue has completely finished, clear the flag and permit regular first-layer learning.

The first XY extrusion can establish the model layer at Z=0.35 mm without an intervening Z move. A layer at the former 0.352 mm exclusion height is also eligible. Manual lines use [explicit SenseOffset activation after priming](automatic-start-line.md#manual-start-lines).

The previous print's spatial fade thresholds can still affect the generated prime when Z homing is skipped. [Z homing lifecycle](../z-offset/lifecycle-homing-fix.md)

## Cache/restore ledger

| State | Action at entry | Action at exit |
| --- | --- | --- |
| Digit Flow profile | Save, override limits with 1500/4500 or S/P and set speed to −90% and extrusion to −75% | Restore |
| M82/M83 E-coordinate mode | Save, force absolute | Restore |
| Logical slicer E coordinate | Save incoming value, use local E0 for prime | Restore incoming E unless print was cancelled |
| `extrudeMultiplyErrorSteps` | Save planner sub-step E remainder | Restore on normal/skip completion |
| `compensatedPositionCollectTinyE` | Save ECMP fractional E remainder under interrupt protection | Restore on normal/skip completion |
| DMS `feedrateFactor()` and `extrusionFactor()` | Continuously adapt to DMS force | Reset both to 1.0 |
| DMS `intervalScale()` | Computed from the DMS speed factor | Reset to 0 atomically |
| Planned layer tracking in `Printer::layers` | Reset tracking and exclude prime movement from learning | Reset tracking |
| `g_startLineActive` | Set before first prime move | Clear after queue drain |
| User `feedrateMultiply`, `menuExtrusionFactor`, `Printer::feedrate` | **Never modified** | Unchanged |
| Manual offset, existing SenseOffset, matrix, `g_min/g_max`, XY/Z position | **Preserve** | Unchanged by reset logic |

**Logical E example:** if the slicer has already reached absolute **E10**, the internal prime can physically push many additional millimetres of filament using its own temporary E0-based coordinates. Afterwards the firmware **restores logical E10**; the next slicer `E11` requests only 1 mm. With conventional entry E0, restoring E0 behaves like the original start code. On cancellation the stop handler's intentional E0 reset is preserved; play-key skip is treated as successful cleanup without extra Y travel.

## Bed compensation and Digit-Z

Bed compensation uses the scanned matrix. The nominal 0.35 mm prime is about 20 µm into the fade at the default `g_min` of 0.33 mm. Geometric ECMP and instantaneous Digit-Z force correction can both apply during priming.

M3912 and Digit Flow are compiled together on all three models.

## Tests performed and remaining tests

**AVR builds passed** for RF1000, RF2000 and RF2000v2 with Arduino AVR core 1.8.6 and avr-gcc 7.3.0. The RF2000v2 build bypassed its existing acknowledgement `#error` in a temporary copy.

**Planner checks passed** for generated priming with a stale limit, a model starting at the same Z as the prime after an E-only unretract, real layers across the former 0.352 mm exclusion window, a descent from a manual line, and a Z hop back to the current layer. Both bundled start-code samples place `M3909` after `M3912`.

**Arithmetic cases passed:** E10→M3912→E11 requests +1 E; E0→M3912→E1 requests +1 E. At DMS 1500 the speed/E factors are 1.0/1.0, at 3000 the speed **goal**/E factor are 0.55/0.625, and at ≥4500 they are 0.10/0.25. Actual speed ramps toward the goal rather than jumping.

The host replay runs the production M3912 case with pressure braking and checks cleanup on completion, play-key skip and print cancellation. [Replay](../../tests/pause-sense-offset.py)

**Physical tests pending.** Check high- and low-force priming, live M220/M221 overrides, M82/M83 and nonzero absolute E, play-key skip and print cancellation, first model layers at 0.20, 0.35, 0.352 and 0.40 mm, and consecutive prints with and without Z homing. Use the motor sound and DMS logs to check the physical brake response.

## Source

[M3912](../../Repetier/RF.cpp#L5273) · [Digit Flow controller](../../Repetier/DigitCompensation.h) · [SenseOffset](sense-offset.md) · [first-layer planner](../../Repetier/motion.cpp) · [coordinate frames and ECMP](coordinate-frames-and-extrusion.md).
