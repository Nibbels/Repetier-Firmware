# M3912: pressure-responsive automatic priming/start line

## Intent

The priming line fills/cleans the nozzle before the first **model** layer. Heavy priming can intentionally generate high DMS force. `M3912` temporarily activates Digit Flow Compensation so excessive measured force causes **slower movement and less extrusion**, rather than treating priming as an incorrect first-layer gap.

The priming line is *not* uncompensated: it still gets the normal scanned-bed correction. It uses the special Z=**0.352 mm** to avoid being learned as the real first layer and to stay above the default 0.33 mm SenseOffset region.

## G-code and generated moves

Example `M3912 I2 E20 F18`. Implementation checks that all axes are homed and bed Z compensation is enabled.

| Option | Meaning | Default / bounds |
| --- | --- | --- |
| `I` | Count of back-and-forth lines | 2; clamps 1..5 |
| `E` | Scaled priming extrusion quantity (not raw absolute E) | 20; clamps 10..30 |
| `F` | Nominal priming line feedrate | 15 mm/s; clamps 5..50 |
| `S`, `P` | DMS lower and upper response thresholds | 1500 / 4500 digits; overrides require both |
| `Y` | Starting Y location | Default 23 mm, bounded by printer geometry |

Before priming, the command waits for the move queue; saves current flow/feed compensation settings and E-coordinate mode; temporarily sets Digit Flow strengths to **-75% extrusion** and **-90% speed**, using the above DMS thresholds.

It then generates alternating lines and advances E. Each line is broken up with a `0.0025` fractional increment (approximately **400 queue segments per line**, not '100' as the old code comment states). This allows new segments to incorporate updated pressure-dependent extrusion multipliers. Feedrate adjustment is applied at runtime in the stepper timing path, not only to newly enqueued segments. The play key and command-block path can interrupt the priming sequence.

After waiting for the queued prime moves, `M3912` restores previous Digit Flow settings, sets the dynamic factors to 1.0, resets E reference and restores E coordinate mode. It also clears `queuePositionZLayerLast` and `Current` so the real model layer can be detected.

## How pressure changes flow vs speed

**Material:** `readStrainGauge()` updates `dynamicExtrusionFactor` depending on the measured DMS force between `Fmin` and `Fmax`. `PrintLine::prepareQueueMove()` multiplies newly planned E increments by that factor. With -75% intensity, maximal response aims at **25% of nominal extrusion**.

**Speed:** `readStrainGauge()` moves `dynamicFeedrateFactor` toward the pressure-dependent target by 0.01 per update. Its reciprocal is converted to `interval_mod`, applied to **queued XY-primary** timer intervals during stepping. With -90% intensity, maximal response aims at **10% of nominal speed**.

The adaptations depend on DMS processing and queue timing; they are not guarantees of a precise physical flow or pressure.

**Critical code gate:** pressure-based Digit Flow effects in `readStrainGauge()` are enabled only when `Printer::currentSteps[Z_AXIS] > g_minZCompensationSteps`. With the baseline `g_min=0.33 mm` and prime `Z=0.352 mm`, this works. If `g_min` has persisted above 0.352 mm from a previous print, pressure-adaptive priming can be blocked **while SenseOffset's opposite Z<=g_min gate becomes eligible**. This is the central lifecycle hypothesis.

## Distinctions to preserve
- Priming line: **pressure → reduce speed / reduce E**.
- True first model layer: **pressure above limit → bounded increase of clearance via SenseOffset**.
- Heat-bed matrix applies in both cases; adaptive first-layer height must refer to the **model**, not the priming line.
- Normal emergency pause/stop thresholds are additional independent protections.
- A genuine model layer near 0.352 mm risks matching the existing magic-height exclusion; explicit semantic priming tagging may eventually be safer.

## Source
- [M3912 generation, overrides, save/restore](../../Repetier/RF.cpp#L9085-L9255)
- [DMS → E/feed factors](../../Repetier/RF.cpp#L319-L398)
- [E factor at planner time](../../Repetier/motion.cpp#L118-L138)
- [Feed factor at ISR timer interval](../../Repetier/motion.cpp#L1625-L1671)
- [Prime-height exclusion](../../Repetier/motion.cpp#L202-L219)
- [Original changelog explanation](../../changelog.txt#L1005-L1013)
