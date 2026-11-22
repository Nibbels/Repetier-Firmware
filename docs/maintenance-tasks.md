# Firmware fixes and candidate pulls

Firmware changes are tracked on `ai-firmware-review` against `community_development`. The [development roadmap](development-roadmap.md) defines the block outcomes and architecture. [Branch comparison](https://github.com/Nibbels/Repetier-Firmware/compare/community_development...ai-firmware-review)

## On the review branch

| Change | Firmware behavior and commits |
| --- | --- |
| DRV8711 quiet settings | The October 2025 import brought in DennisNochmal's OFF and DECAY writes. The review branch removes the BLANK read and uses the reset default. Afriess made the same quiet-mode commands selectable at compile time before that import. [Import](https://github.com/Nibbels/Repetier-Firmware/commit/4e8467d) · [BLANK fix](https://github.com/Nibbels/Repetier-Firmware/commit/8f00ed8) · [Afriess switch](https://github.com/afriess/RFx000x-Firmware/commit/2f0b9a3) |
| Z offsets and homing | `M3006` includes the applied SenseOffset when editing the saved manual offset. Z homing resets learned layer geometry and the live SenseOffset. Disabling bed compensation clears stale SenseOffset and pressure history. [M3006](https://github.com/Nibbels/Repetier-Firmware/commit/b4ae1dc) · [Z homing](https://github.com/Nibbels/Repetier-Firmware/commit/ba93708) · [Compensation off](https://github.com/Nibbels/Repetier-Firmware/commit/733833a) |
| Pressure-braked `M3912` priming | An explicit priming phase replaces the 0.352 mm marker on all three models and keeps the generated line out of first-layer detection. Priming preserves the user's feed and extrusion overrides, restores the logical E position and its fractional step accounting, and clears the temporary timer slowdown. [Priming state](https://github.com/Nibbels/Repetier-Firmware/commit/5f848c9) · [Timer reset](https://github.com/Nibbels/Repetier-Firmware/commit/b793cd8) · [Priming phase](https://github.com/Nibbels/Repetier-Firmware/commit/b0aa720) |
| Pause, resume and jog | Standby cooldown uses the correct temperature units. The pause handler serializes park and return. It computes return travel from the saved direct XYZ position and keeps the queue paused if the return is blocked. Jog release also works when another key is held. [Cooldown and endstops](https://github.com/Nibbels/Repetier-Firmware/commit/0c8d899) · [Pause and jog](https://github.com/Nibbels/Repetier-Firmware/commit/5883673) |
| Planned steps and relative moves | XYZ moves use integer endpoint differences. Relative G-code retains the rounding remainder, and E retains a separate fractional motor-step carry. The long relative-Z replay went from 288 steps off to zero, and 100,000 small relative E moves produced 224,000 steps. [XYZ planner](https://github.com/Nibbels/Repetier-Firmware/commit/0a5bdd9) · [Relative coordinates](https://github.com/Nibbels/Repetier-Firmware/commit/5883673) · [Replay details](first-layer-control/coordinate-frames-and-extrusion.md#why-segment-rounding-cannot-accumulate-in-xyz) |

## Coordinate systems and motion compensation

**Active development block.** [State and motion responsibilities](development-roadmap.md#1-coordinate-systems-and-motion-compensation)

### Z travel and scanning

- [x] **Allow scan retreat from the exact lower bound.** `moveZ()` stops a further approach at `-maxZOverrideSteps` and allows the bed to move away. The scan keeps control of the Z motor for the duration of its move. [Scan step output](../Repetier/RF.cpp#L3332)

- [x] **Apply travel limits to compensation-only Z pulses.** Each pulse checks the motor position in the requested direction. Z-max uses the configured limit and, where present, the hardware switch. Z-min allows the model-specific overtravel after homing. An unreachable target stays pending, leaving the next retreat step available. Host replays covered idle and XY movement, queued and direct retreat, and disabling compensation at a limit. Physical verification remains open. [Compensation pulses](../Repetier/motion.cpp#L827) · [Travel limit check](../Repetier/Printer.cpp#L904) · [Scan reach](z-offset/scans-and-persistence.md#z-min-screw-adjustment-and-scan-reach)

- [x] **Use the configured Z maximum on the RF1000.** The RF1000 homes on its physical Z-min switch and limits travel toward Z-max in software. The factory setting is 195 mm and the stored Z height takes precedence. RF2000 and RF2000v2 use their separate Z-min and Z-max inputs. [Endstop configurations](development-roadmap.md#feature-retirement)

### Pause and resume

- [ ] **Suspend automatic SenseOffset learning during pause.** Bed-matrix lookup and fade use the frozen print XYZ. In a host replay, pre-pause samples raised SenseOffset by 10 µm after the reading fell to zero. Stop learning and clear sample history while retaining applied clearance, manual offset adjustment and live Digit-Z force tracking. The pause guard passed a host trial. Check the return at equal force and after manual extrusion while parked. [Print matrix coordinates](../Repetier/RF.cpp#L2857) · [Pressure handler](../Repetier/RF.cpp#L4019)

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

## Print lifecycle and DMS controls

### Z lifecycle

- [ ] **Reset learned fade bounds for prints started without Z homing.** The previous print's bounds can remain active into the next print, including during priming. Identify a reliable print-start boundary that also covers jobs without `M3912`. [Lifecycle finding](z-offset/lifecycle-and-findings.md#finding-f3-partially-fixed-consecutive-print-lifecycle)

### Digit-Z

- [ ] **Recover the same elastic correction at the same steady DMS reading.** Retain the integer filter remainder to eliminate the one-microstep difference reproduced at 5,530 digits. Verify rising and falling force, then measure the force-to-deflection slope and the delay from load change to applied Z on the printer. [Formula, timing and replay](z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)

## Calibration and persistence

- [ ] **Make the `M3902 Z0` offset transfer durable.** The command saves zero for the manual offset immediately. A reset before saving the RAM bed matrix can lose the transferred correction. [Matrix persistence](z-offset/scans-and-persistence.md#critical-transfer-operation-m3902-z0)

## Motion smoothness and performance

- [ ] **Evaluate Z-CMP wait between queued segments.** The review branch skips waits for corrections held at a travel limit and keeps idle compensation active with a paused queue. Compare the fork's change to waits between buffered moves in the timing review. [Timer patch](https://github.com/afriess/RFx000x-Firmware/commit/e51f9bc)
- [ ] **Evaluate step packing.** Check the computed `minInterval` with the extra 64 ticks for pressure advance. Measure removing the 512-tick interval floor separately. [Step-packing patch](https://github.com/afriess/RFx000x-Firmware/commit/0e11752)

## Firmware structure and hardware variants

Work through the [architecture review and migration](development-roadmap.md#5-firmware-structure-and-hardware-variants) as the blocks establish which state, algorithms and hardware differences belong together.

- [x] **Retire milling mode.** Removed the mode switch, workpiece scans and compensation, along with their commands, UI pages and configuration. Preserved the print EEPROM layout. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Retire ten disabled features.** Removed ditto printing, position memory, standalone parking, secondary X/Y/Z motor drivers, service reminders, caliper input, viscosity testing and the INT3 test counter. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Use one print build configuration.** Compiled the eight DMS and print features on all models, removed arc and servo commands, and selected RGB by the board pin mapping. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Retire sound and fix the build configuration for Digit Homing and movement unlock.** Both print functions compile on all three models. Boot diagnostics use the serial port for a board-type mismatch. [Feature retirement](development-roadmap.md#feature-retirement)
- [ ] **Review remaining optional features for retirement.** Check actual usage, dependencies and the simplification from removing the alternate paths.

## Fork candidates

- [Z-CMP timing and step packing](#motion-smoothness-and-performance)
