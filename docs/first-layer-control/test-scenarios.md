# Validation matrix and open questions

**M3912 implemented patch:** [priming lifecycle and regression tests](m3912-state-boundary.md), [firmware commit 5f848c9](https://github.com/Nibbels/Repetier-Firmware/commit/5f848c9a5cd0347c6d45fd9d3f13afd7344cb775). Physical tests pending.

Also review the [Z-offset lifecycle risk register](../z-offset/lifecycle-and-findings.md). The M3006 omission of accumulated SenseOffset was **fixed in [b4ae1dc](https://github.com/Nibbels/Repetier-Firmware/commit/b4ae1dc53a184c7a866db987f4419413912a58d5)** on the review branch; M3909 P0 retaining the existing correction and cross-print state remain open for validation.

This is **analysis, not confirmation from a physical printer**.

## Invariants

- The RF1000/RF2000/RF2000v2 share first-layer and DMS logic. Keep general fixes general.
- Prime at 0.352 mm must follow bed scan but **not** count as first model layer. Strong prime force should activate feed/material reduction, **not** SenseOffset lift.
- True model first layer, whichever positive height, should determine learned full-bed-following limit. SenseOffset may then increase gap within its max.
- Fade-out and **ECMP E steps** must remain consistent and not exhibit discontinuities.
- SenseOffset can only increase its bounded offset; lowering force must not drive nozzle closer.
- Distinguish *planner-ahead* layer updates from *currently executed Z*.
- No new expensive work in stepper ISR or unbounded delays; run hardware tests conservatively.

## Suspected faults (not yet demonstrated)

**H1: previous-print threshold leak WITHOUT Z homing.** A previous print can retain stale geometric `g_min/g_max`. **Generated M3912 now forces the correct DMS brake and suspends SenseOffset** even with stale limits; a new Z home is still required to reset the **geometric** compensation boundaries.

**H2: layer-state cleanup.** `homeZAxis()` and the patched `M3912` now clear `Current`, `Last` and `GuessNew`. Generated priming moves cannot be accepted as model layers. Motor-disable-only paths still do not reset all learned geometric bounds.

**H3: wrong fix scope/time.** [r3dacto fork patch](https://github.com/r3dacto/Repetier-FirmwareCodex/commit/d59ca5e53c1351c47a9ab67606806968c82f976e) clears `GuessNew` then resets fade bounds after M3912 **only on RF2000v2**. That would occur *after* a potentially wrong prime-force response. We found no special RF2000v2 dependency in shared layer logic.

**H4: numerical sentinel.** A real first model layer at 0.352 mm (±5 steps) could be incorrectly excluded from dynamic boundary learning. Likewise, manual start lines at another height are not semantically tagged.

**H5: geometry and mode.** Sequential objects, prints without M3912, changing tools, manual offsets and printer-specific Z origins may challenge the present height checks. The `g_max` dynamic formula assumes a valid positive fade span; validate sign/zero handling and ECMP denominators.

## Suggested bench/print scenarios

| Scenario | Verify |
| --- | --- |
| Fresh home + M3001 + M3912 + model Z0.20 | Pressure controls prime feed/E only; real first layer learned at 0.20; SenseOffset available there |
| Same with model Z0.40 | 0.352 mm prime excluded, then valid 0.40 mm first-layer boundary |
| Print A Z0.40 → **G28 Z** → Print B Z0.20 | Fresh geometric bounds and force-braked prime; model layer correctly relearned |
| Print A Z0.40 → Print B Z0.20 **without** Z homing | Prime DMS policy correct, but old g_min/g_max geometry may remain |
| Genuine model first layer Z0.352 | Known rare sentinel collision: it remains excluded by legacy numerical logic |
| Play-key skip in M3912 | No extra Y move; E timeline, Digit Flow policy and layer markers restored/cleared |
| Print cancellation during M3912 | Stop further prime moves; reset phase and pressure policy; preserve stop-handler E0 |
| M3909 P0 and M3000 | Correct controller deactivation and offset behavior |
| Force above threshold in model first layer | Positive-only, capped SenseOffset adjustment |
| Force above threshold in priming | Feed/E reduction without unintended bed lift |
| Traverse min and max fade boundaries | Continuous Z target plus appropriate material correction |
| Manual prime, sequential object, tool change | No false first-layer transition or stale guess |

### Patch strategy

First instrument or simulate state transitions for `Current`, `Last`, `GuessNew`, `g_min`, `g_max`, current Z, DMS force, active SenseOffset and Digit Flow factors. Do not reset learned bounds after an active layer was identified. Decide separately when to reset *layer markers* and *learned compensation boundaries*; do not conflate them. Change one lifecycle event at a time and test all shared models/configurations. Keep live EEPROM settings in view: compiled defaults may not be active on a real printer.
