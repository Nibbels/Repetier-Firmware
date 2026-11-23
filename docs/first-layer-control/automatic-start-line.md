# M3912: pressure-responsive automatic priming/start line

See the [M3912 state ledger and verification results](m3912-state-boundary.md).

**Coordinate reference and verified limitations:** [G-code Z, physical Z compensation and E logical timeline](coordinate-frames-and-extrusion.md). Normal speed override and DMS speed control are separate.

## Intent

**Mechanical interpretation:** high DMS force produces an audible, simulated **braking effect**—like a chainsaw entering wood—because firmware progressively lowers the XY stepping speed and, as necessary, E flow. This is not an indication that the stepper motors are out of power. The intent is to relieve nozzle load **without learning a new Z-offset from priming force**.

The priming line fills and clears the nozzle before the first model layer. `M3912` temporarily activates Digit Flow Compensation so high measured force causes **slower movement and less extrusion**.

Generated priming runs at nominal Z=0.35 mm with scanned-bed compensation. The runtime phase flag identifies its moves. The former 0.352 mm exclusion was chosen to distinguish priming from common first-layer heights. Any otherwise eligible model layer can now be learned at that height.

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

After waiting for the queued prime moves, `M3912` restores the previous Digit Flow profile and calls `resetFlow()` to neutralize both live factors and the timer interval scale. It restores the **incoming logical E reference** and both sub-step E remainders on completion or skip. It restores the E mode and resets layer tracking so the real model layer can be detected. A skip finishes the queued prime moves before cleaning up the phase.

## How pressure changes flow vs speed

**Material.** `DigitCompensation` updates its extrusion factor from the filtered DMS force and the profile limits. `PrintLine::prepareQueueMove()` multiplies newly planned E increments by that factor. With −75% intensity, maximal response aims at **25% of nominal extrusion**.

**Speed.** `DigitCompensation` moves its feedrate factor toward the pressure-dependent target by 0.01 per update. It publishes the reciprocal as a fixed-point scale for **queued XY-primary** timer intervals. With −90% intensity, maximal response aims at **10% of nominal speed**. [Filter and factors](../../Repetier/DigitCompensation.h)

The adaptations depend on DMS processing and queue timing; they are not guarantees of a precise physical flow or pressure.

Outside `M3912`, Digit Flow uses `currentZ > g_min` and SenseOffset uses `currentZ <= g_min`. During generated priming the flag enables Digit Flow eligibility and pauses SenseOffset correction regardless of `g_min`. Without Z homing, previous `g_min/g_max` values can still affect the scanned-bed fade during priming.

## Distinctions to preserve
- Priming line: **pressure → reduce speed / reduce E**.
- True first model layer: **pressure above limit → bounded increase of clearance via SenseOffset**.
- Heat-bed matrix applies in both cases; adaptive first-layer height must refer to the **model**, not the priming line.
- Normal emergency pause/stop thresholds are additional independent protections.

## Manual start lines

Place `M3909` with a positive `P` threshold after the final priming move and before the model starts. The enable task runs in motion-queue order. Turn off AutoStart in **Configuration → DMS Features → SenseOffset** to keep `M3001` from enabling it before the line. To override AutoStart from G-code, put `M400` and then `M3909 P0` after `M3001` and before the manual prime.

The layer detector sees manual priming as regular extrusion. A manual line above the model's first layer lets the descent to that layer establish the new compensation boundary.

## Source
- [M3912 generation, overrides, save/restore](../../Repetier/StartLine.cpp)
- [DMS → E/feed factors](../../Repetier/RF.cpp#L319-L398)
- [E factor at planner time](../../Repetier/motion.cpp#L118-L138)
- [Feed factor at ISR timer interval](../../Repetier/motion.cpp#L1625-L1671)
- [Phase flag and first-layer detection](../../Repetier/motion.cpp#L170-L203)
- [Original changelog explanation](../../changelog.txt#L1005-L1013)
