# Open firmware tasks

## Pause and resume

- [ ] **Define how Z compensation behaves while paused.** SenseOffset checks the frozen queued first-layer Z. It can keep accumulating after a direct park move has opened the nozzle gap. The return restores the direct Z position while compensation can still change the motor position. Compare queued Z, direct Z and applied compensation before pause, while parked and before the queue restarts. Include manual extrusion while parked and preserve the existing clearance and saved manual offset. [SenseOffset gate](../Repetier/RF.cpp#L5688-L5726) · [Z coordinate contributions](../Repetier/RF.h#L438-L475)

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
