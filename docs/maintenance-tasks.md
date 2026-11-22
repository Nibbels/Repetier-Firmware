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

### Z offset ownership

- [x] **Give the effective Z offset one owner.** `Printer::zOffset` owns the manual and live values and calculates their combined steps. A reload through `M501` or `M206` preserves the live contribution. The host regression covers reload, Mod Menu transfer, clearing the live portion, and changes to steps per mm. [Offset ledger and regression command](z-offset/offset-ledger.md)

### Layer geometry and fade boundaries

- [x] **Give planned layer tracking and fade boundaries one owner.** `Printer::layers` handles layer detection, resets and fade scaling. The host replay compares consecutive prints, model extrusion at the priming height, a Z hop, and correction and ECMP values across the fade. It also exercises the reset and scale operations used after EEPROM load and microstep changes. [Layer state](../Repetier/PrintLayers.h) · [Host replay](../tests/pause-sense-offset.py)

The four AVR builds matched the previous revision's CPU cycles and Z step counts in nine instruction traces per build. These cover idle compensation, queued X and Z, opposing Z correction and direct moves. Flash changed by −30 bytes on RF1000, +60 on RF2000 and RF2000v2, and +66 on the RF2000 X44 retrofit. RF1000 uses four fewer bytes of global SRAM. Physical tests remain open.

### Z travel and scanning

- [x] **Give Z step execution one owner.** `ZMotion` tracks nominal progress, direct displacement, scan travel, requested and applied correction, missed queued steps and emitted motor steps. The host replay covers endstop clipping, overtravel repayment, opposing correction, compensation waits, scan retreat and microstep scale changes. Run `python3 tests/z-motion.py` for all three model policies. [Execution and limits](development-roadmap.md#z-step-execution)

- [x] **Allow scan retreat from the exact lower bound.** A homed scan stops further approach at the configured Z-min overtravel limit and can still drive the bed away. [Scan step output](../Repetier/ZMotion.cpp) · [Scan reach](z-offset/scans-and-persistence.md#z-min-screw-adjustment-and-scan-reach)

- [x] **Apply travel limits to compensation-only Z pulses.** Each pulse checks the motor position in the requested direction. Z-max uses the configured limit and, where present, the hardware switch. Z-min allows the model-specific overtravel after homing. An unreachable target stays pending, leaving the next retreat step available. Host replays cover idle correction, queued and direct retreat, and disabling compensation at a limit. Physical verification remains open. [Compensation pulses and limits](../Repetier/ZMotion.cpp)

- [x] **Use the configured Z maximum on the RF1000.** The RF1000 homes on its physical Z-min switch and limits travel toward Z-max in software. The factory setting is 195 mm and the stored Z height takes precedence. RF2000 and RF2000v2 use their separate Z-min and Z-max inputs. [Endstop configurations](development-roadmap.md#feature-retirement)

### Wobble correction

- [x] **Keep the WobbleFix offset out of the retained G-code target.** Previously, each absolute `G1 Y…` added another 100 µm to X with an X amplitude of 100 µm at machine Z=0 and phase zero. The planner now adds the correction to the step endpoint. In a host replay, all 50 Y-only moves retained the same X after the first corrected target. Relative moves, `G92`, E-only moves, homing resets and software limits also passed. Physical verification remains open. [Correction](../Repetier/Printer.cpp#L255) · [Step endpoint](../Repetier/Printer.cpp#L285)

### Pause and resume

- [x] **Give the pause and stop sequences explicit state owners.** The replay runs the production transitions for panel Pause and Play, queued admission, clipped XY/Z/E returns, an overlapping jog release, cancellation during parking, reheating and returning, and timed stop cleanup. It also exercises standby and stop across clock rollover. Run `python3 tests/pause-sense-offset.py`. Physical verification remains open. [Pause and stop transitions](development-roadmap.md#pause-and-stop-transitions)

- [x] **Suspend automatic SenseOffset learning during pause.** The previous handler could add 10 µm of clearance from pre-pause samples after the DMS reading fell to zero. A pause now invalidates the learning window as soon as it is accepted. The host replay tests park, jog, manual extrusion, reheating, return at equal force, a pause between polls and a queued pause during the offset update. It also verifies live Digit-Z response. Physical verification remains open. [SenseOffset during pause](first-layer-control/sense-offset.md#pause-and-resume)

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

- [x] **Initialize first-layer state with Z homing.** The host replay covers Z homing between first layers of 0.40 mm and 0.20 mm, a fresh SenseOffset sample window, and model extrusion without `M3912`. Physical verification remains open. [USB print start](first-layer-control/README.md#usb-print-start)

### Digit-Z

- [ ] **Recover the same elastic correction at the same steady DMS reading.** Retain the integer filter remainder to eliminate the one-microstep difference reproduced at 5,530 digits. Verify rising and falling force, then measure the force-to-deflection slope and the delay from load change to applied Z on the printer. [Formula, timing and replay](z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)

## Calibration and persistence

- [ ] **Make the `M3902 Z0` offset transfer durable.** The command saves zero for the manual offset immediately. A reset before saving the RAM bed matrix can lose the transferred correction. [Matrix persistence](z-offset/scans-and-persistence.md#critical-transfer-operation-m3902-z0)

## Motion smoothness and performance

- [ ] **Review Pressure Advance.** Compare `advanceL=0` with a nonzero per-tool value from `M233 Y…` or EEPROM. Trace queued and direct E steps, pending advance steps and their timer through accelerations, decelerations, pauses, tool changes and print cancellation. Check total E pulses and remaining advance steps at the end of each run before evaluating step packing. [Advance calculation](../Repetier/motion.cpp#L420) · [E timer](../Repetier/HAL.cpp#L957)
- [ ] **Review the junction-speed formula.** Replay straight joins at equal and unequal feedrates, sharp corners and reversals, plus mixed XYZ/E and pure E. Trace the XY, Z and E limits through lookahead to the resulting junction speed. [Junction planner](../Repetier/motion.cpp#L586)
- [ ] **Measure the slowdown on short segments.** Use repeated direction changes and equivalent paths with different segment lengths around 0.6 mm. Compare measured speed, queue depth and pulse spacing. Evaluate the distance weighting alongside the junction-speed limits. [Junction planner](../Repetier/motion.cpp#L586)
- [ ] **Evaluate Z-CMP wait between queued segments.** The review branch skips waits for corrections held at a travel limit and keeps idle compensation active with a paused queue. Compare the fork's change to waits between buffered moves in the timing review. [Timer patch](https://github.com/afriess/RFx000x-Firmware/commit/e51f9bc)
- [ ] **Evaluate step packing.** Check the computed `minInterval` with the extra 64 ticks for pressure advance. Measure removing the 512-tick interval floor separately. [Step-packing patch](https://github.com/afriess/RFx000x-Firmware/commit/0e11752)

## Firmware structure and hardware variants

Work through the [architecture review and migration](development-roadmap.md#5-firmware-structure-and-hardware-variants) as the blocks establish which state, algorithms and hardware differences belong together.

- [x] **Retire milling mode.** Removed the mode switch, workpiece scans and compensation, along with their commands, UI pages and configuration. Preserved the print EEPROM layout. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Retire ten disabled features.** Removed ditto printing, position memory, standalone parking, secondary X/Y/Z motor drivers, service reminders, caliper input, viscosity testing and the INT3 test counter. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Use one print build configuration.** Compiled the eight DMS and print features on all models, removed arc and servo commands, and selected RGB by the board pin mapping. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Retire sound and fix the build configuration for Digit Homing and movement unlock.** Both print functions compile on all three models. Boot diagnostics use the serial port for a board-type mismatch. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Fold board outputs and core controls into the model builds.** Removed guided nozzle alignment and the optional driver status command. AVR builds cover all three models. A host replay of the bed scan completion states covers the standard, PLA and ABS paths through matrix save. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Remove the remaining `FEATURE_*` switches.** Use model temperature curves, compile homing diagnostics and WobbleFix on all models, and remove the unused checksum bit. Host tests compared sensor conversion and zero-amplitude move targets. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Fold the shared motion, display and serial defaults into the firmware.** Pressure Advance, the current junction and short-segment calculations, the 25 ms driver enable wait and the 120 s menu timeout are built in. Automatic trace builds, alternate serial ports and the early RF1000 fan-pin variant are retired. [Shared build defaults](development-roadmap.md#feature-retirement)
- [x] **Retire TipDown and fix the UART transmit ring at 64 bytes.** Removed the per-tool Z displacement from tool changes, homing and EEPROM settings. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Retire the M109 retract and obsolete build variants.** The heat-up wait leaves extrusion to the print's G-code. Board selection determines the physical Z-max input. The heater and LCD use their stock variants. [Feature retirement](development-roadmap.md#feature-retirement)
- [x] **Share the RF2000 family profile and pin map.** Compile the stock defaults for each model, keep named connectors for retrofits and reserve X19, X42 or X44 for automatic cooling when assigned there. EEPROM support is built in with the existing persisted layout. [Board reference](board-connectors.md)

### SD support removal

Commit `0ae25f2` contains the complete SD removal. It deletes the card driver and SdFat library, card settings and SPI support, file commands `M20`–`M30` and `M32`, the LCD browser and SD-specific status, and the G-code source switching. All three models receive external G-code through USB and use a 24-move planner queue. Use the removal diff and its parent revision to investigate or reimplement SD later. [Removal commit](https://github.com/Nibbels/Repetier-Firmware/commit/0ae25f2a57b8e68f6dc734ff6f849962581a9ee4) · [Complete source before removal](https://github.com/Nibbels/Repetier-Firmware/tree/330726479462569312a0e35114e260236a3dbde5)

```sh
git show 0ae25f2a57b8e68f6dc734ff6f849962581a9ee4
git show 0ae25f2a57b8e68f6dc734ff6f849962581a9ee4^:Repetier/SDCard.cpp
```

The former SD path used AVR hardware SPI with CS on digital pin 53 (PB0), SCK on 52 (PB1), MOSI on 51 (PB2), and MISO on 50 (PB3). RF2000 and RF2000v2 used active-low card detection on pin 19 (PD2). RF1000 had no card-detect input. SdFat used table-driven CRC, single-block I/O and a shared FAT16/FAT32 cache. The filename buffer used two VFAT entries and the folder depth was two. [Archived pin map](https://github.com/Nibbels/Repetier-Firmware/blob/330726479462569312a0e35114e260236a3dbde5/Repetier/pins.h) · [Archived settings](https://github.com/Nibbels/Repetier-Firmware/blob/330726479462569312a0e35114e260236a3dbde5/Repetier/Configuration.h#L323) · [Archived SdFat code](https://github.com/Nibbels/Repetier-Firmware/blob/330726479462569312a0e35114e260236a3dbde5/Repetier/SdFat.cpp)

The old file source called `close()` on normal EOF and entered `Printer::stopPrint()`. A host replay on `351dd60` discarded three buffered moves and the active move. If SD is reimplemented, verify that normal EOF drains queued commands and motion, including a final line without a newline and pause and resume during draining. Test read errors and cancellation separately. [Archived file source](https://github.com/Nibbels/Repetier-Firmware/blob/330726479462569312a0e35114e260236a3dbde5/Repetier/gcode.cpp#L1120)

## Fork candidates

- [Z-CMP timing and step packing](#motion-smoothness-and-performance)
