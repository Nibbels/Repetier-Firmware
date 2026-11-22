# Firmware fixes and candidate pulls

Firmware changes are tracked on `ai-firmware-review` against `community_development`. [Branch comparison](https://github.com/Nibbels/Repetier-Firmware/compare/community_development...ai-firmware-review)

## On the review branch

| Change | Firmware behavior and commits |
| --- | --- |
| DRV8711 quiet settings | The October 2025 import brought in DennisNochmal's OFF and DECAY writes. The review branch removes the BLANK read and uses the reset default. Afriess made the same quiet-mode commands selectable at compile time before that import. [Import](https://github.com/Nibbels/Repetier-Firmware/commit/4e8467d) · [BLANK fix](https://github.com/Nibbels/Repetier-Firmware/commit/8f00ed8) · [Afriess switch](https://github.com/afriess/RFx000x-Firmware/commit/2f0b9a3) |
| Z offsets and homing | `M3006` includes the applied SenseOffset when editing the saved manual offset. Z homing resets learned layer geometry and the live SenseOffset. Disabling bed compensation clears stale SenseOffset and pressure history. [M3006](https://github.com/Nibbels/Repetier-Firmware/commit/b4ae1dc) · [Z homing](https://github.com/Nibbels/Repetier-Firmware/commit/ba93708) · [Compensation off](https://github.com/Nibbels/Repetier-Firmware/commit/733833a) |
| Pressure-braked `M3912` priming | An explicit priming phase replaces the 0.352 mm marker on all three models. Priming preserves the user's feed and extrusion overrides, restores the logical E position and its fractional step accounting, and clears the temporary timer slowdown. [Priming state](https://github.com/Nibbels/Repetier-Firmware/commit/5f848c9) · [Timer reset](https://github.com/Nibbels/Repetier-Firmware/commit/b793cd8) · [Priming phase](https://github.com/Nibbels/Repetier-Firmware/commit/b0aa720) |
| Pause, resume and jog | Standby cooldown uses the correct temperature units. The pause handler serializes park and return. It computes return travel from the saved direct XYZ position and keeps the queue paused if the return is blocked. Jog release also works when another key is held. [Cooldown and endstops](https://github.com/Nibbels/Repetier-Firmware/commit/0c8d899) · [Pause and jog](https://github.com/Nibbels/Repetier-Firmware/commit/5883673) |
| Planned steps and relative moves | XYZ moves use integer endpoint differences. Relative G-code retains the rounding remainder, and E retains a separate fractional motor-step carry. The long relative-Z replay went from 288 steps off to zero, and 100,000 small relative E moves produced 224,000 steps. [XYZ planner](https://github.com/Nibbels/Repetier-Firmware/commit/0a5bdd9) · [Relative coordinates](https://github.com/Nibbels/Repetier-Firmware/commit/5883673) · [Replay details](first-layer-control/coordinate-frames-and-extrusion.md#why-segment-rounding-cannot-accumulate-in-xyz) |

## Fork candidates

| Candidate | Disposition |
| --- | --- |
| Z-CMP wait between queued segments | Adapt alongside the Z travel fix. The fork's timer change also stops idle compensation during a pause with buffered moves. Preserve live idle compensation and let a move away from a travel limit proceed when a correction is blocked. [Timer patch](https://github.com/afriess/RFx000x-Firmware/commit/e51f9bc) |
| Step-packing threshold | Evaluate in the performance review. Using the computed `minInterval` honors the extra 64 ticks for pressure advance. Removing the 512-tick interval floor is a separate motion timing change to evaluate. [Step-packing patch](https://github.com/afriess/RFx000x-Firmware/commit/0e11752) |
| RF2000v2 post-priming layer reset | The shared priming phase covers the brake and layer tracking during the generated start line. Consecutive prints without Z homing can still inherit fade bounds. [Fork patch](https://github.com/r3dacto/Repetier-FirmwareCodex/commit/d59ca5e53c1351c47a9ab67606806968c82f976e) · [Lifecycle finding](z-offset/lifecycle-and-findings.md#finding-f3-partially-fixed-consecutive-print-lifecycle) |
| Auto-touch and workpiece origin | Deferred. The experimental probing routines refine workpiece contact and origin measurement. The 0.25 to 0.50 mm change widens the origin search. [Probe routine](https://github.com/afriess/RFx000x-Firmware/blob/0e11752/Repetier/RF.cpp#L3382-L3405) · [Search range](https://github.com/afriess/RFx000x-Firmware/commit/0dd35e4) |

## Z travel and scanning

- [ ] **Allow scan retreat from the exact lower bound.** A direction check in `moveZ()` passed a host trial for probing past the switch, reaching `-maxZOverrideSteps`, blocking further approach and retreating from that bound. The firmware patch is pending. [Scan step output](../Repetier/RF.cpp#L4789-L4828)

- [ ] **Apply travel limits to compensation-only Z pulses.** Host replays emitted pulses past Z-max and past the lower Z-min override between target refreshes. Check the actual motor position and configured limit before each pulse, preserving the model-specific travel past the homing trigger. Allow retreat even when compensation is waiting at a limit. Check idle and XY motion. [Compensation pulses](../Repetier/motion.cpp#L969-L1036) · [Existing Z endstop rules](../Repetier/motion.cpp#L1290-L1361) · [Scan reach](z-offset/scans-and-persistence.md#z-min-screw-adjustment-and-scan-reach)

## Pause and resume

- [ ] **Suspend automatic SenseOffset learning during pause.** In print mode, bed-matrix lookup and fade use the frozen print XYZ. In a host replay, pre-pause samples raised SenseOffset by 10 µm after the reading fell to zero. Stop learning and clear sample history while retaining applied clearance, manual offset adjustment and live Digit-Z force tracking. The pause guard passed a host trial. Check the return at equal force and after manual extrusion while parked. [Print matrix coordinates](../Repetier/RF.cpp#L3198-L3221) · [SenseOffset sampling](../Repetier/RF.cpp#L5688-L5726)

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

## Digit-Z

- [ ] **Recover the same elastic correction at the same steady DMS reading.** Retain the integer filter remainder to eliminate the one-microstep difference reproduced at 5,530 digits. Verify rising and falling force, then measure the force-to-deflection slope and the delay from load change to applied Z on the printer. [Formula, timing and replay](z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)

## Z lifecycle

- [ ] **Handle a new print without Z homing.** A second print can inherit the previous layer's fade bounds. Identify a reliable job boundary before changing the calibration state. [Lifecycle finding](z-offset/lifecycle-and-findings.md#finding-f3-partially-fixed-consecutive-print-lifecycle)
- [ ] **Make the `M3902 Z0` offset transfer durable.** The command saves zero for the manual offset immediately. A reset before saving the RAM bed matrix can lose the transferred correction. [Matrix persistence](z-offset/scans-and-persistence.md#critical-transfer-operation-m3902-z0)
