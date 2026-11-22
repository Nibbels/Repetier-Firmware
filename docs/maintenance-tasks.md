# Open firmware tasks

## Pause and resume

- [ ] **Suspend automatic SenseOffset learning during pause.** In print mode, bed-matrix lookup and fade already use the frozen print XYZ. SenseOffset still accepts readings at that first-layer Z. In a host replay, pre-pause samples raised the offset by 10 µm after the current reading fell to zero. Sustained simulated force could reach the default 180 µm cap. Use the existing pause state to stop learning and clear sample history while retaining the applied offset. Preserve manual offset adjustments and keep Digit-Z force tracking active. Compare the first-layer return at equal force and include manual extrusion while parked. [Print matrix coordinates](../Repetier/RF.cpp#L3198-L3221) · [SenseOffset sampling](../Repetier/RF.cpp#L5688-L5726) · [Digit-Z target](../Repetier/RF.cpp#L3271-L3293)

- [ ] **Apply travel limits to compensation-only Z pulses.** Host replays emitted pulses past Z-max and past the lower Z-min override between target refreshes. Check the allowed direction against the physical limit at pulse time. Preserve travel from the homed Z-min trigger to the configured lower bound and allow movement away from either limit. Check idle and XY motion. [Compensation pulse output](../Repetier/motion.cpp#L969-L1036) · [Z move endstop handling](../Repetier/motion.cpp#L1290-L1361) · [Scan reach](z-offset/scans-and-persistence.md#z-min-screw-adjustment-and-scan-reach)

- [ ] **Allow scan retreat from the exact lower bound.** A host replay of `moveZ()` advanced through the intended window, then rejected a retreat pulse at `-maxZOverrideSteps` before checking direction. Verify contact past the switch, a blocked approach at the configured bound, and travel away from that bound during scan recovery. [Scan step output](../Repetier/RF.cpp#L4789-L4828)

- [x] **Serialize pause transitions and restore the saved direct step position.** A single handler owns the park and return moves. It saves the three direct XYZ positions before parking. Play returns the difference between the saved and current steps, then checks each part of the return before releasing the print queue. A blocked return stays paused so the next attempt can travel only the remaining distance. Direct moves preserve the queue's separate accounting for steps ignored at travel limits. [Pause handler](../Repetier/RF.cpp#L6189-L6275) · [Limit handling](../Repetier/motion.cpp#L1209-L1358)

- [x] **Stop a held jog when its initiating key is released.** The fast UI handler trims the jog when the selected key changes. Resuming also trims an active held jog before calculating the return. [Release check](../Repetier/ui.cpp#L5196-L5220) · [Resume guard](../Repetier/RF.cpp#L6424-L6454)

The updated host replay covers early Pause and Play on pressure, queued and panel requests, ordinary and overlapping jog releases, a blocked return and retry, cancellation, and the two-press wake and resume sequence. The step replay covers clipped direct moves, preservation of queued overtravel, and direct extrusion with the pressure guard. AVR builds passed for all three models. Physical verification remains open.

### Original timing replay on `0c8d899`

The host replay used the firmware's pause handlers, foreground UI, and key polling with simulated timer ticks and motor movement. Park requests used the RF2000 defaults with room for the full move.

| Input | Observed motion |
| --- | --- |
| Play during the retract in a pressure pause | Resume released the queue, then the original handler issued Z+30 mm and Y+200 mm. Both direct offsets remained applied. |
| Play during the Z lift in a pressure pause | Resume returned Z and released the queue, then the original handler issued Y+200 mm and left it applied. |
| Play before a queued park initializes the return offsets | The return used a stale offset from a previous 40 mm Y jog. |
| Physical Pause before a queued in-place pause initializes the return offsets | The button advanced directly to the park stage. A later Play left a 20 mm Y error from an earlier jog. |
| Physical Play during the Z lift after a G-code panel Pause (`M3029 P1518`) | The G-code dispatch bypassed the panel's UI action guard. The original park left Y+200 mm after resume. |
| Play during a park started with the physical Pause button | The foreground UI guard blocked the overlapping Play. A Play after parking returned to the starting point. |
| Play held across release of Bed DOWN during a settled pause | The downward jog continued to the simulated Z maximum before the return began. |

The firmware restricted Play to a completed pause in 1.37t2. The 1.43.56 pause rewrite removed that check. The 1.43.76 standby change added checks for heating and the initial return request. [1.37t2](https://github.com/Nibbels/Repetier-Firmware/commit/c9b3a88) · [1.43.56](https://github.com/Nibbels/Repetier-Firmware/commit/391cedb) · [1.43.76](https://github.com/Nibbels/Repetier-Firmware/commit/6f5167d)

## G-code coordinates

- [x] **Preserve precision when accumulating long relative moves.** Relative updates retain the small remainder of each float addition. The Z stress replay that previously left 288 steps now returns to zero, and 100,000 small relative E commands reach the expected 224,000 steps. [Endpoint proof and replay](first-layer-control/coordinate-frames-and-extrusion.md#why-segment-rounding-cannot-accumulate-in-xyz)

## Digit-Z

- [ ] **Recover the same elastic correction at the same steady DMS reading.** Retain the integer filter remainder to eliminate the one-microstep difference reproduced at 5,530 digits. Verify rising and falling force, then measure the force-to-deflection slope and the delay from load change to applied Z on the printer. [Formula, timing and replay](z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)
