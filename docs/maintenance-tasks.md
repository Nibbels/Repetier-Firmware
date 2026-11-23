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

### Commanded coordinates

- [x] **Give the requested position and work offset one owner.** The G-code replay checks absolute work coordinates with an offset, relative moves, inches, omitted axes and E-only moves. [Commanded coordinates](../Repetier/MoveTarget.h) · [Replay](../tests/pause-sense-offset.py)
- [x] **Give queued XYZ step endpoints one owner.** The replay checks 100,000 relative X increments, a calibration change from 160 to 320 steps/mm at the same target and a return to zero. [XYZ step planning](../Repetier/CartesianPlanner.h) · [Replay](../tests/pause-sense-offset.py)
- [x] **Share segment geometry between queued and direct motion.** Both paths use the same signed step setup, primary-axis selection and distance calculation. The replay covers equal step counts in multiple axes, mixed XYZ/E and an E-only retract. [Segment geometry](../Repetier/motion.cpp) · [Replay](../tests/pause-sense-offset.py)
- [x] **Initialize the task field of every warm-up entry.** A queue replay discarded 23 commands and then wrapped the four warm-up entries through slot zero. Previously the reused slot still held a discarded M3000 task, which the dispatcher would run before inspecting the warm-up flag. Now all four entries contain only a warm-up. [Queue warm-up](../Repetier/motion.cpp)
- [x] **Give the motion buffer one owner.** `MoveQueue` owns the slots, indices, occupancy and active entry. The replay covers publishing, cancellation, wraparound, temporary release while waiting and interrupt-protected retirement. [Motion buffer](../Repetier/motion.h) · [Replay](../tests/pause-sense-offset.py)
- [x] **Keep the direct segment inside the motion executor.** Timer1, pause, panel moves and print stop query its state through dedicated methods. The DMS stop check reads direct and queued Z activity in one interrupt-protected snapshot. The replay covers active direct Z, queued Z and clearing the queue. [Motion executor](../Repetier/motion.cpp) · [Replay](../tests/pause-sense-offset.py)
- [x] **Keep the step clocks in the motion executor.** The direct and queued paths retain their own elapsed ticks, step count and peak speed. One routine advances the clocks and handles acceleration, cruise and deceleration. The replay checks phase transitions, the minimum exit speed and independence of the two clocks. [Step clocks](../Repetier/motion.cpp) · [Replay](../tests/pause-sense-offset.py)

### Z offset ownership

- [x] **Give the effective Z offset one owner.** `Printer::zOffset` owns the manual and live values and calculates their combined steps. A reload through `M501` or `M206` preserves the live contribution. The host regression covers reload, Mod Menu transfer, clearing the live portion, and changes to steps per mm. [Offset ledger and regression command](z-offset/offset-ledger.md)

### Layer geometry and fade boundaries

- [x] **Give planned layer tracking and fade boundaries one owner.** `Printer::layers` handles layer detection, resets and fade scaling. The host replay compares consecutive prints, model extrusion at the priming height, a Z hop, and correction and ECMP values across the fade. It also exercises the reset and scale operations used after EEPROM load and microstep changes. [Layer state](../Repetier/PrintLayers.h) · [Host replay](../tests/pause-sense-offset.py)

The four AVR builds matched the previous revision's CPU cycles and Z step counts in nine instruction traces per build. These cover idle compensation, queued X and Z, opposing Z correction and direct moves. Flash changed by −30 bytes on RF1000, +60 on RF2000 and RF2000v2, and +66 on the RF2000 X44 retrofit. RF1000 uses four fewer bytes of global SRAM. Physical tests remain open.

- [x] **Extract the bed correction and ECMP calculation.** The foreground supplies nominal Z, layer geometry and a matrix reader to `BedCompensation`. The replay checks target steps, ECMP and matrix reads at negative and zero Z, both fade boundaries, a flat matrix and disabled compensation. The active Digit-Z replay covers both sensor polarities and the 600-step limit. A regression probe reproduces the early ECMP write on negative Z in `0383de5` and checks publication under interrupt protection on the new revision. All four builds use 46 fewer flash bytes, and the 36 instruction traces match the previous cycles and Z step counts. Physical tests remain open. [Calculation](../Repetier/BedCompensation.h) · [Host replay](../tests/pause-sense-offset.py)

### XY travel limits

- [x] **Share the XY limit rule and own the skipped steps.** The replay checks both axes, either limit, repayment with the switch held and after it opens, and the difference between clipped queued and direct moves. [Accounting](../Repetier/XYMotion.h) · [Replay](../tests/pause-sense-offset.py)
- [x] **Give executed XY coordinates one owner.** `XYMotion` holds nominal steps, motor steps, the direct displacement and skipped queue steps for both axes. Homing, step output, compensation and reporting use the matching coordinate through its methods. [XY execution positions](../Repetier/XYMotion.h) · [Replay](../tests/pause-sense-offset.py)

Across the four AVR builds, 64 XY instruction traces reproduce the motor displacements and skipped-step counts from `92f2b10`. An unobstructed queued X step takes one more CPU cycle and the corresponding Y step takes the same number. Limit paths range from ten fewer to six more cycles. Physical pulse timing remains open.

### Z travel and scanning

- [x] **Give the Z-offset scan named phases.** The scan owner keeps the current phase, probe Z and the leveling route. Each service call advances one phase and keeps the existing calibration waits. The replay checks normal completion, an unstable pressure retry, a missing EEPROM matrix and all seven leveling points. [Z-offset scan replay](../tests/z-offset-scan.py)
- [x] **Give the bed mesh scan named phases.** The scan owner keeps the mesh cursor, timestamps and retry phase. The standard, PLA, ABS, retry and cancel replays match the old scan's phase counts and verify the saved mesh and heater targets. [Bed scan replay](../tests/heat-bed-scan.py)

- [x] **Give Z step execution one owner.** `ZMotion` tracks nominal progress, direct displacement, scan travel, requested and applied correction, missed queued steps and emitted motor steps. The host replay covers endstop clipping, overtravel repayment, opposing correction, compensation waits, scan retreat and microstep scale changes. Run `python3 tests/z-motion.py` for all three model policies. [Execution and limits](development-roadmap.md#z-step-execution)

- [x] **Allow scan retreat from the exact lower bound.** A homed scan stops further approach at the configured Z-min overtravel limit and can still drive the bed away. [Scan step output](../Repetier/ZMotion.cpp) · [Scan reach](z-offset/scans-and-persistence.md#z-min-screw-adjustment-and-scan-reach)

- [x] **Apply travel limits to compensation-only Z pulses.** Each pulse checks the motor position in the requested direction. Z-max uses the configured limit and, where present, the hardware switch. Z-min allows the model-specific overtravel after homing. An unreachable target stays pending, leaving the next retreat step available. Host replays cover idle correction, queued and direct retreat, and disabling compensation at a limit. Physical verification remains open. [Compensation pulses and limits](../Repetier/ZMotion.cpp)

- [x] **Use the configured Z maximum on the RF1000.** The RF1000 homes on its physical Z-min switch and limits travel toward Z-max in software. The factory setting is 195 mm and the stored Z height takes precedence. RF2000 and RF2000v2 use their separate Z-min and Z-max inputs. [Endstop configurations](development-roadmap.md#feature-retirement)

### Extrusion planning

- [x] **Give the planned E reference and base-step carry one owner.** The replay uses the production destination update and planner arithmetic. It checks `G92` with a fractional origin, a half-step reversal with a flow multiplier and 100,000 relative 0.008 mm moves, which yield 224,000 steps at 280 steps/mm. The priming replay checks restoration of the base-step carry. [Planner](../Repetier/ExtrusionPlanner.h) · [Host replay](../tests/pause-sense-offset.py)

- [x] **Encapsulate geometric extrusion compensation during execution.** The replay checks base and extra pulses, the intermediate falling edge, direct extrusion and the advance queue. [Execution state](../Repetier/ExtrusionCompensation.h) · [Host replay](../tests/pause-sense-offset.py)
- [x] **Balance the ECMP remainder through a retract and its matching unretract.** The original replay lost one E step with a ratio of 0.25 and a starting remainder of 0.75. It now produces −1/+1 steps and restores the remainder. The replay also checks −2/+2 at a remainder of 0.125 and −105/+105 for 100 base steps each way at 4.8% ECMP. Both stepper pulse output and Pressure Advance pass. Across the four AVR builds, the shortest sampled ECMP pulses had 47 cycles high and 103 cycles low at 16 MHz. Physical verification remains open. [Host replay](../tests/pause-sense-offset.cpp) · [DRV8711 timing](https://www.ti.com/lit/ds/symlink/drv8711.pdf#page=9)
- [x] **Give Pressure Advance step accounting one owner.** `ExtrusionOutput` owns the pending E steps, requested lead, direction filter and idle release countdown. The replay checks both directions, the compare-register wrap, lead updates and release after 70 idle ticks. [State](../Repetier/ExtrusionOutput.h) · [Timer](../Repetier/HAL.cpp) · [Replay](../tests/pause-sense-offset.py)
- [x] **Give the separate extruder timer a 2 µs STEP pulse.** The zero-delay review build (`78bb1de`) produced 16 or 27 high cycles, and `acc5d30` added 32 cycles to each pulse. The timed output now selects the tool before the rising edge and books the step during the high phase. The AVR trace verifies 32 high cycles at 16 MHz for each fitted tool in four builds. The full interrupt takes 119 cycles on single-extruder builds and 136–140 on dual-extruder builds. These are 13–17 cycles less than the matching cases in `78bb1de`. [Timer](../Repetier/HAL.cpp) · [Timed output](../Repetier/HAL.h) · [DRV8711 timing](https://www.ti.com/lit/ds/symlink/drv8711.pdf#page=9)

The pressure protection, sensor, start-line and extrusion-planner extractions together use 368–410 fewer flash bytes and 3 more bytes of global SRAM than `777525c`. The final 36 instruction traces reproduce the previous Z step counts and cycles. Physical verification remains open.

### Wobble correction

- [x] **Give the WobbleFix profile and cached offsets one owner.** The replay checks 50 successive Y targets, Y amplitude interpolation across X, a clipped correction retained for the next move and the transition to zero amplitudes. [Wobble correction](../Repetier/WobbleCorrection.h) · [Replay](../tests/pause-sense-offset.py)

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

- [x] **Give generated priming one owner.** The replay runs `M3912` through completion, skip and cancellation and checks its admission, parameter limits, generated moves and restoration of the flow and E state. The layer and SenseOffset replays use the actual priming phase. [Start line](../Repetier/StartLine.cpp) · [State ledger](first-layer-control/m3912-state-boundary.md)

### SenseOffset

- [x] **Give SenseOffset learning one owner.** The replay covers the ten-sample window, both force polarities, a falling average, tapering near the cap, pause, priming and crossing the first-layer boundary. It exercises queued settings activation, `M3909 P0`, Mod Menu thresholds and disabling bed compensation. It also checks that a queued limit change invalidates an increment that was calculated before the new limits arrived. The four AVR builds use 1 fewer byte of global SRAM and 22–32 more flash bytes. The 36 stepper instruction traces match the previous cycles and Z step counts. Physical verification remains open. [SenseOffset control](../Repetier/SenseOffsetControl.h) · [Host replay](../tests/pause-sense-offset.py)

### Strain gauge and Digit-Z

- [x] **Extract sensor access and Digit Homing.** The driver configures and reads the ADC and owns the zero offset. The replay checks the signed I²C reading, fresh zero measurement, enable and disable, the raw-reading limits and rejection at 5000 digits. [Sensor driver](../Repetier/StrainGauge.cpp) · [Host replay](../tests/pause-sense-offset.py)

- [x] **Give the DMS filter and Digit Flow one owner.** The host replay checks the sampling schedule, filter output, first-layer gating, live Z target and both flow factors. It runs M3912 through completion, skip and cancellation. The four AVR builds retain the same instruction sequence for applying the interval scale and mask both byte writes when publishing it. The 36 stepper traces match the previous cycles and Z step counts. Global SRAM drops by 3 bytes per build. Physical verification remains open. [Digit compensation](../Repetier/DigitCompensation.h) · [Host replay](../tests/pause-sense-offset.py)

- [x] **Recover the same elastic correction at the same steady DMS reading.** The DMS filter retains its division remainder. The replay now settles at 54 Z microsteps for a reading of 5,530 digits both after a rise and after a fall in load. It checks both sensor polarities. [Filter](../Repetier/DigitCompensation.h) · [Host replay](../tests/pause-sense-offset.py)
- [ ] **Measure Digit-Z on the printer.** Record the force-to-deflection slope and the delay from load change to applied Z. [Formula, timing and measurement](z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)

### Pressure protection

- [x] **Give the DMS emergency windows one owner.** The replay exercises `M3075` and `M3076`, the saved limits, the exact-boundary samples, the homing gate, emergency park, the Z stop and the extrusion inhibit. The four AVR builds match the prior 36 step traces. [Pressure protection](../Repetier/PressureProtection.h) · [Host replay](../tests/pause-sense-offset.py)
- [x] **Decide DMS Z stop from the executing move.** Read the active axes with the same direct-move priority and pause gate as Timer1. A pressure fault inhibits E and requests Z stop for an executing move with Z and no E axis. Host replays cover mixed Z/E, Z with a cached E direction and paused queued Z with direct E on both tool-count builds. Physical verification remains open. [DMS decisions](../Repetier/RF.cpp) · [Motion snapshot](../Repetier/motion.cpp)
- [x] **Report the temporary stop band set by `M3076 S0 P0`.** The status now names the active [0, 0] band. The replay checks the message, the inhibit for a nonzero average, and retention of the saved EEPROM limits. [G-code handler](../Repetier/RF.cpp) · [Replay](../tests/pause-sense-offset.cpp)

## Calibration and persistence

- [ ] **Make the `M3902 Z0` offset transfer durable.** The command saves zero for the manual offset immediately. A reset before saving the RAM bed matrix can lose the transferred correction. [Matrix persistence](z-offset/scans-and-persistence.md#critical-transfer-operation-m3902-z0)

## Motion smoothness and performance

Use the [throughput goal, timing analysis and test matrix](motion-timing.md) for this block.

- [ ] **Measure straight and curved print workloads end to end.** Report actual step and segment rates, buffered milliseconds, priority service gaps, UART hardware overruns and ring drops, and the longest duration with interrupts masked. Use the same streamed geometry for each comparison. [Timing baseline](motion-timing.md#performance-measurements-and-acceptance)
- [x] **Finish the Advance output handoff.** Complete the old E balance at tool and timer changes, including a held single step. Discard pending work on pressure inhibit, cancel and shutdown. Pause retries the portion of the return that never reached the motor. [STEP ownership and replays](motion-timing.md#step-and-direction-ownership)
- [ ] **Replace idle Advance polling with on-demand compare events.** Preserve the E interval and pulse timing through wake, cancel, an isolated pending step and compare wrap. Exercise heater PWM and Arduino timekeeping while COMPA sleeps. [Implementation order](motion-timing.md#design-and-implementation-order)
- [x] **Separate lookahead data from step execution.** The foreground planner prepares queued and direct step profiles in the existing slots. The replay covers queue wrap, fixed extrusion joins, a matching step interval trace and jog release before and during movement. The queue still uses 2,861 bytes in all four AVR builds. The linker retains two existing 16-bit scan variables that were optimized out in the baseline, increasing total global SRAM by four bytes. [Planner and executor boundary](motion-timing.md#planner-and-executor-boundary)
- [ ] **Measure Pressure Advance on the printer.** Compare `advanceL=0` and nonzero per-tool settings during acceleration, deceleration, print–travel joins, E-only moves and the handoff cases. Capture both tools' STEP and DIR signals, pulse counts, direction hold and setup, compare-wrap latency and the time from a DMS trip to the last edge. [Advance calculation](../Repetier/MotionPlanner.cpp) · [E timer](../Repetier/HAL.cpp)
- [ ] **Review the junction-speed formula.** Replay straight joins at equal and unequal feedrates, sharp corners and reversals. Trace the XY, Z and E limits through lookahead to the resulting junction speed. [Junction planner](../Repetier/MotionPlanner.cpp#L213)
- [ ] **Measure short-move pacing and stutter.** Replay equivalent curves with segment lengths around 0.6 mm under steady and bursty delivery. Track the queue-count limiter and the geometric junction limit separately from blocked execution and an empty queue. Compare speed and pulse spacing at the same buffered time. [Planner rules](motion-timing.md#lookahead-short-moves-and-extrusion-transitions)
- [ ] **Review extrusion transitions in a dedicated planner pass.** Trace print–travel–print and E-only retract and unretract with Advance on and off. Record which rule fixes each join, pressure lead, E steps and nozzle behavior before deciding how much speed to retain. [Current extrusion joins](motion-timing.md#lookahead-short-moves-and-extrusion-transitions)
- [ ] **Evaluate Z-CMP wait between queued segments.** The review branch skips waits for corrections held at a travel limit and keeps idle compensation active with a paused queue. Compare the fork's change to waits between buffered moves in the timing review. [Timer patch](https://github.com/afriess/RFx000x-Firmware/commit/e51f9bc)
- [ ] **Evaluate step packing.** Check the computed `minInterval` with the extra 64 ticks for pressure advance. Measure removing the 512-tick interval floor separately. [Step-packing patch](https://github.com/afriess/RFx000x-Firmware/commit/0e11752)

## Firmware structure and hardware variants

Work through the [architecture review and migration](development-roadmap.md#5-firmware-structure-and-hardware-variants) as the blocks establish which state, algorithms and hardware differences belong together.

- [x] **Separate planning, stepping and move completion.** `StepExecutor` owns timer state, driver direction and the panel feedrate snapshot. It finishes moves through a single path that retires the queue slot or releases direct offsets. `MotionPlanner` supplies the read-and-clear Advance peak for `M232`. [Executor](../Repetier/StepExecutor.cpp) · [Timing comparison](motion-timing.md#planner-and-executor-boundary)
- [x] **Specialize serial transport and isolate command input.** `UsbSerial` owns USART0 and its rings. `GCodeInput` owns the session, framing and queue while `GCode` carries the parsed parameters. The transcripts cover ASCII, binary, display text, backpressure, resend and draining during print stop. [Compatibility contract](development-roadmap.md#serial-printing-compatibility)
- [x] **Own the Ready deadline and the shutdown timeouts.** Both handle clock rollover. The Ready transition is published under interrupt protection. Shutdown settings use the existing EEPROM entries and menu and G-code paths. [Ready replay](../tests/pause-sense-offset.py) · [Inactivity replay](../tests/machine-inactivity.cpp)

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
