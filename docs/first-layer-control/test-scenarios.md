# Validation matrix and open questions

**M3912 implemented patch:** [priming lifecycle and regression tests](m3912-state-boundary.md), [firmware commit 5f848c9](https://github.com/Nibbels/Repetier-Firmware/commit/5f848c9a5cd0347c6d45fd9d3f13afd7344cb775). Physical tests pending.

Also review the [Z-offset lifecycle risk register](../z-offset/lifecycle-and-findings.md). The M3006 omission of accumulated SenseOffset was **fixed in [b4ae1dc](https://github.com/Nibbels/Repetier-Firmware/commit/b4ae1dc53a184c7a866db987f4419413912a58d5)** on the review branch; M3909 P0 retaining the existing correction and cross-print state remain open for validation.

This is **analysis, not confirmation from a physical printer**.

## Invariants

- The RF1000/RF2000/RF2000v2 share first-layer and DMS logic. Keep general fixes general.
- Generated priming at nominal Z=0.35 mm uses scanned-bed compensation, suspends layer learning and SenseOffset, and responds to force with the DMS speed and E brake.
- The first model layer below 1 mm determines the learned full-bed-following limit when detected. SenseOffset can then increase clearance up to its cap.
- Fade-out and **ECMP E steps** must remain consistent and not exhibit discontinuities.
- SenseOffset can only increase its bounded offset; lowering force must not drive nozzle closer.
- Distinguish *planner-ahead* layer updates from *currently executed Z*.
- No new expensive work in stepper ISR or unbounded delays; run hardware tests conservatively.

## Suspected faults (not yet demonstrated)

**H1: previous-print threshold leak.** A fresh Z home or `M3921` before priming restores the geometric `g_min/g_max` defaults. The generated `M3912` phase forces the DMS brake and suspends SenseOffset during priming, independently of the learned layer.

**H2: layer-state cleanup.** `homeZAxis()` and the patched `M3912` now clear `Current`, `Last` and `GuessNew`. Generated priming moves cannot be accepted as model layers. Motor-disable-only paths still do not reset all learned geometric bounds.

**H3: wrong fix scope/time.** [r3dacto fork patch](https://github.com/r3dacto/Repetier-FirmwareCodex/commit/d59ca5e53c1351c47a9ab67606806968c82f976e) clears `GuessNew` then resets fade bounds after M3912 **only on RF2000v2**. That would occur *after* a potentially wrong prime-force response. We found no special RF2000v2 dependency in shared layer logic.

**H4: manual start-line ordering.** Keep SenseOffset disabled until after the manual prime. With a manual line above the model's first layer, the descent lets the planner relearn the limit. A line below the model's first layer can be learned as the first layer.

**H5: geometry and mode.** Sequential objects, prints without M3912, changing tools, manual offsets and printer-specific Z origins may challenge the present height checks. The `g_max` dynamic formula assumes a valid positive fade span; validate sign/zero handling and ECMP denominators.

## Suggested bench/print scenarios

| Scenario | Verify |
| --- | --- |
| Fresh home + M3001 + M3912 + model Z0.20 | Pressure controls prime feed/E only; real first layer learned at 0.20; SenseOffset available there |
| Same with model Z0.40 | Generated prime bypasses layer learning. The model learns the 0.40 mm boundary |
| Print A Z0.40 → **G28 Z** → Print B Z0.20 | Fresh geometric bounds and force-braked prime; model layer correctly relearned |
| Print A Z0.40 → `M3921` → Print B Z0.20 with the original home reference | Default fade bounds during priming, then a 0.20 mm first-layer boundary |
| `M3921` → model XY extrusion at Z0.352 with a valid home reference | Learn the first model layer without an automatic prime or a separate Z move |
| A pause arrives while `M3921` waits for queued motion | The marker reports a blocked start and the paused print retains its layer and offset state |
| Genuine model first layer Z0.352 and neighbouring Z steps | Normal first-layer learning across the former exclusion window |
| Model first layer Z0.35 immediately after M3912 | Without a Z step, the first XY extrusion sets the 0.35 mm boundary |
| Manual prime above the first layer, with AutoStart off and `M3909` after priming | SenseOffset activates after the prime. Descending to the first layer establishes its boundary |
| Play-key skip in M3912 | No extra Y move; E timeline, Digit Flow policy and layer markers restored/cleared |
| Print cancellation during M3912 | Stop further prime moves; reset phase and pressure policy; preserve stop-handler E0 |
| M3909 P0 and M3000 | Correct controller deactivation and offset behavior |
| Force above threshold in model first layer | Positive-only, capped SenseOffset adjustment |
| Force above threshold in priming | Feed/E reduction without unintended bed lift |
| Traverse min and max fade boundaries | Continuous Z target plus appropriate material correction |
| Manual prime, sequential object, tool change | No false first-layer transition or stale guess |

### Patch strategy

First instrument or simulate state transitions for `Current`, `Last`, `GuessNew`, `g_min`, `g_max`, current Z, DMS force, active SenseOffset and Digit Flow factors. Do not reset learned bounds after an active layer was identified. Decide separately when to reset *layer markers* and *learned compensation boundaries*; do not conflate them. Change one lifecycle event at a time and test all shared models/configurations. Keep live EEPROM settings in view: compiled defaults may not be active on a real printer.
