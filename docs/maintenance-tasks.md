# Open firmware tasks

## Pause and resume

- [ ] **Define how Z compensation behaves while paused.** SenseOffset checks the frozen queued first-layer Z. It can keep accumulating after a direct park move has opened the nozzle gap. The return motion reverses the direct Z offset while compensation can still change the motor position. Compare queued Z, direct Z and applied compensation before pause, while parked and before the queue restarts. Include manual extrusion while parked and preserve the existing clearance and saved manual offset. [SenseOffset gate](../Repetier/RF.cpp#L5688-L5726) · [Z coordinate contributions](../Repetier/RF.h#L438-L475)

- [ ] **Serialize Pause and Play with the park and return moves.** The PWM timer samples the panel keys, and the foreground UI can dispatch them from the loop inside a blocking direct move. A pressure pause or a G-code pause can start outside the panel's UI action guard. Play currently accepts both a pending pause and an active park, and Pause can advance an in-place pause to the park stage before the return offsets have been reset. Give the transition exclusive control of those offsets and hold the print queue until the return completes. Include the normal panel sequence and the two-press standby sequence in regression checks. [Key handling](../Repetier/ui.cpp#L5093-L5220) · [Direct-move waits](../Repetier/Printer.cpp#L586-L636) · [Button guards](../Repetier/RF.cpp#L6369-L6452)

- [ ] **Stop a held jog when its initiating key is released.** The fast UI handler trims a stoppable jog only when it samples no key. If Play stays down while Bed DOWN is released, the jog can continue toward Z max while the return waits for it to finish. Check both an ordinary release and a release with a second key held. [Key priority](../Repetier/ui.cpp#L581-L594) · [Release check](../Repetier/ui.cpp#L5191-L5219) · [Paused Z jog](../Repetier/RF.cpp#L9966-L9991)

### Timing replay on `0c8d899`

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
