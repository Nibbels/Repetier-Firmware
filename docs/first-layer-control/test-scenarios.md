# Validation matrix and open questions

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

**H1: previous-print threshold leak.** A prior 0.40 mm first layer may leave `g_min=0.40 mm`. A new M3912 at 0.352 mm would then be **below** the SenseOffset gate and **not above** the Digit Flow gate. The M3912 and Z reset paths we inspected do not restore the learned `g_min`/`g_max`. Determine precisely when a print/homing/compensation reset should reinitialize them.

**H2: incomplete layer-state reset.** M3912 clears `Current` and `Last` but not `GuessNew`. Z-stepper disable/reset also leaves `GuessNew` untouched. Establish whether queued Z moves can leave a stale candidate despite queue-drain waits.

**H3: wrong fix scope/time.** [r3dacto fork patch](https://github.com/r3dacto/Repetier-FirmwareCodex/commit/d59ca5e53c1351c47a9ab67606806968c82f976e) clears `GuessNew` then resets fade bounds after M3912 **only on RF2000v2**. That would occur *after* a potentially wrong prime-force response. We found no special RF2000v2 dependency in shared layer logic.

**H4: numerical sentinel.** A real first model layer at 0.352 mm (±5 steps) could be incorrectly excluded from dynamic boundary learning. Likewise, manual start lines at another height are not semantically tagged.

**H5: geometry and mode.** Sequential objects, prints without M3912, changing tools, manual offsets and printer-specific Z origins may challenge the present height checks. The `g_max` dynamic formula assumes a valid positive fade span; validate sign/zero handling and ECMP denominators.

## Suggested bench/print scenarios

| Scenario | Verify |
| --- | --- |
| Fresh home + M3001 + M3912 + model Z0.20 | Pressure controls prime feed/E only; real first layer learned at 0.20; SenseOffset available there |
| Same with model Z0.40 | 0.352 mm prime excluded, then valid 0.40 mm first-layer boundary |
| Print A Z0.40 → Print B Z0.20 without reboot | No inherited g_min during second prime; second first layer learned correctly |
| Genuine first layer Z0.352 | No persistent exclusion of model layer |
| Abort M3912 using play key | Pressure controls and extrusion mode restored; queued moves resolved |
| M3909 P0 and M3000 | Correct controller deactivation and offset behavior |
| Force above threshold in model first layer | Positive-only, capped SenseOffset adjustment |
| Force above threshold in priming | Feed/E reduction without unintended bed lift |
| Traverse min and max fade boundaries | Continuous Z target plus appropriate material correction |
| Manual prime, sequential object, tool change | No false first-layer transition or stale guess |

### Patch strategy

First instrument or simulate state transitions for `Current`, `Last`, `GuessNew`, `g_min`, `g_max`, current Z, DMS force, active SenseOffset and Digit Flow factors. Do not reset learned bounds after an active layer was identified. Decide separately when to reset *layer markers* and *learned compensation boundaries*; do not conflate them. Change one lifecycle event at a time and test all shared models/configurations. Keep live EEPROM settings in view: compiled defaults may not be active on a real printer.
