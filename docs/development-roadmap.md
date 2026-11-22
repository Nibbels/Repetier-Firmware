# RFx000 firmware development roadmap

The current block is **coordinate systems and motion compensation**. The aim is to explain every requested and applied movement from G-code through the motor pulses, then simplify the firmware around those responsibilities. The work covers the shared firmware for RF1000, RF2000 and RF2000v2. The [maintenance list](maintenance-tasks.md) records individual fixes, candidates and verification results.

| Development block | Outcome |
| --- | --- |
| [Coordinate systems and motion compensation](#1-coordinate-systems-and-motion-compensation) | **Active.** One accountable path for nominal geometry, direct motion, compensation and motor position. |
| [Print lifecycle and DMS controls](#2-print-lifecycle-and-dms-controls) | Explicit job and phase transitions, with consistent pressure control and reset behavior. |
| [Calibration and persistence](#3-calibration-and-persistence) | A traceable path from measured contact to the active and saved calibration. |
| [Motion smoothness and performance](#4-motion-smoothness-and-performance) | A timing baseline and measured improvements. Step packing belongs here. |
| [Firmware structure and hardware variants](#5-firmware-structure-and-hardware-variants) | Fewer supported combinations and clear boundaries between machine logic, operator interfaces and hardware. |

Use the architecture below throughout the blocks. Calibration can progress independently once the live and saved offset semantics are clear. Evaluate timing changes after the coordinate behavior has a repeatable baseline. Review the scope of optional features before restructuring the paths that serve them.

## Architecture to test against the code

`motion.cpp` currently contains path planning, queued task dispatch, direct moves, endstop decisions and step output. `RF.cpp` combines scan motion, matrix math, pressure control, pause and print state, and special G-code dispatch. `Printer` holds state read and written by these paths. [`Commands.cpp`](../Repetier/Commands.cpp#L54) refreshes compensation in the foreground, and [`HAL.cpp`](../Repetier/HAL.cpp#L739) switches between compensation, direct motion and queued motion in the stepper interrupt. [Planner and executor](../Repetier/motion.cpp) · [RF machine functions](../Repetier/RF.cpp)

### One owner for motion

`Printer::zOffset` owns the saved manual offset, the live SenseOffset and their combined step count. EEPROM reloads preserve the live contribution, and calibration changes recompute steps from micrometers. `Printer::zMotion` owns executed Z positions, the homing state, travel limits and pulse accounting. The planner, DMS and bed calculations submit step requests to it. [Offset owner](../Repetier/ZOffset.h) · [Z motion owner](../Repetier/ZMotion.h)

Route movement and motion state changes through named operations such as queueing a move, jogging, homing, probing, parking, resuming and applying new calibration. G-code and the panel request the same operations where applicable. Give planning, pulse execution, compensation calculation and the print phases their own state owners behind that interface. Sensor sampling and storage supply inputs, and the UI reads snapshots.

Use that same view for firmware diagnostics and host replays. For a suspect move, show the active motion source, nominal target and progress, direct displacement, requested and applied correction, motor step count, and any endstop or inhibition reason. Track which operation changed the state. Sample at the points where this explains a transition, with pulse-level tracing reserved for test builds. This gives future fixes a single place to assert and observe motion behavior.

| Responsibility | State and handoff |
| --- | --- |
| Command and operator input | Interpret G-code units, work origin and coordinate modes. UI, serial and SD submit the applicable machine actions. Keep command parsing, display updates and storage access outside step execution. |
| Nominal motion planning | Own commanded endpoints, queued endpoints and the logical E timeline. Round XYZ to integer step endpoints and carry the established fractional state. Pass a defined segment to the executor and track its nominal progress separately from the motor position. |
| Compensation calculation | Produce targets in declared units from the nominal path, bed matrix, tool and offset state, and DMS inputs. Identify which stage applies bed fade, geometric E compensation (ECMP), Digit Flow, Digit-Z and SenseOffset. Publish a consistent target to step execution. |
| Physical motion execution | Account for motor steps actually emitted, corrections actually applied and planned steps suppressed by an endstop. Make arbitration among queued, direct, scan and compensation moves explicit. Keep the interrupt path bounded and define how foreground work reads or replaces shared state. |
| Print phases and calibration | Own transitions for homing, priming, printing, pause, resume and stop. Define which values reset, survive or are recalculated at each transition. Own calibration changes from measurement through activation and persistent save. |

Test whether `PrintLine` can become a queue entry with execution handled separately. Use a common motion execution interface for scan, homing, jog and print where their rules agree. Express the different rules at that interface, including probing past the Z-min trigger and compensating during paused printing. Use small functions and existing representations where they already express the behavior. [Coordinate frames and extrusion](first-layer-control/coordinate-frames-and-extrusion.md) · [Offset units and operations](z-offset/offset-ledger.md)

### Z step execution

`PrintLine` supplies the Z step request and its path type. `ZMotion` decides whether to emit a pulse, consume a step without a pulse or stop the segment's Z axis. Each emitted pulse updates the motor position through the same operation, including scan and homing diagnostic pulses.

| Motion source | Accounting and limits |
| --- | --- |
| Queued moves | Each due step advances nominal Z. Suppressed steps at Z-max accrue overtravel on the queued path. Returning from the limit consumes that overtravel before further nominal steps emit pulses. A queue move configured to stop at an endstop can terminate its Z axis. |
| Direct moves | Advance the jog or pause displacement only for steps accepted by execution. A clipped move ends at the accepted displacement. A held-button jog stops at Z-min contact. Homed single-step jogs can use the configured Z-min overtravel. |
| Z compensation | Keep the requested and applied correction separately. Positive pulses respect the configured maximum and the Z-max switch where fitted. Negative pulses respect the Z-min switch before homing and the overtravel limit after homing. A reachable opposing correction and a queued or direct step can advance together without a pulse. A correction held at a limit leaves the retreat step available. |
| Scan motion | Record displacement from the scan reference. Compensation pulses wait during a scan move. The approach stops at the homed Z-min overtravel limit and retreat is available at that position. |

Travel calibration updates the maximum, the Z-min overtravel and the quarter-millimetre wait threshold together. Microstep edits therefore preserve the physical overtravel distance. The wait calculation in the timer uses integer steps. [Execution rules](../Repetier/ZMotion.cpp) · [Z motion replay](../tests/z-motion.py)

### Cost on the ATmega2560

A small Z-pulse compilation probe used the repository's AVR-GCC 7.3.0 with `-mmcu=atmega2560 -Os -flto`. It checked the target, motor position and endstop before updating the pulse and two counters. Three standalone volatile 32-bit values and the same three values as private members of a static object produced byte-identical 226-byte Timer1 handlers. Each probe used 528 bytes of `.text` and 12 bytes of `.bss`. A deliberately out-of-line method generated a call and increased the handler's register pushes from 12 to 16. This establishes that the small encapsulated path can compile at the same cost. Complete firmware timing remains to be measured.

Use short, statically bound methods on the interrupt path, including the separate advance timer. Keep coordinate conversion and DMS filtering in the foreground. Transfer only the values needed for the next action, with brief interrupt protection for multi-byte state shared with the foreground. Compare linked AVR assembly, stack use, SRAM and pulse timing on the migrated path. Make the first migration earn its place by reducing the places that can change a value and by exposing the resulting motion state to a replay or trace.

The Z migration compiles with 3 more bytes of global SRAM per model. Compared with `97d0aee`, flash decreases by 12 bytes on RF1000, 34 bytes on RF2000 and RF2000v2, and 86 bytes on the RF2000 X44 retrofit. The linked Timer1 handlers contain no float-to-integer conversion for the Z compensation wait.

On the RF2000 build, an instruction trace of an idle tick with 400 steps of pending correction takes 504 CPU cycles, down from 600. The movement part of a one-step queued X tick takes 441 cycles, up from 435. A queued Z step with an opposing correction takes 372 cycles, up from 344. These traces use fixed inputs and exclude nested interrupts. Measure pulse spacing and timing headroom on the printer.

## 1. Coordinate systems and motion compensation

- **Define the coordinate path.** Trace G-code origin and units through nominal integer endpoints, executed nominal steps, direct or scan offsets, requested and applied compensation, and actual motor steps. Record where tool offsets and endstop-suppressed travel enter. For E, track logical position and base, ECMP and advance pulses. Use the trace to identify duplicate conversions, overlapping writers and places where one frame is read as another.
- **Use the Z motion owner as the baseline for the remaining refactor.** Pause transitions and the saved return position now belong to `Printer::pause`. Give bed and DMS calculations explicit inputs in nominal coordinates and outputs in correction steps. The step executor owns the execution counters. [Z step execution](#z-step-execution) · [Pause and stop transitions](#pause-and-stop-transitions)
- **Handle the pause boundary.** Every accepted pause invalidates the SenseOffset window. Park, jog, reheat and return keep the existing clearance and live Digit-Z, and print motion resumes with a fresh learning window. The host replay checks the return at equal DMS force after manual extrusion. [Pause verification](maintenance-tasks.md#pause-and-resume)
- **Verify movement end to end.** Replay many small absolute and relative moves, tool changes, homing and coordinate resets, clipped motion, and pause/jog/resume. Check integer endpoint and pulse counts, frame changes at each transition, and recovery at both Z limits. On the printer, check scan retreat, bounded compensation and return to the print path. Use the [coordinate replay findings](first-layer-control/coordinate-frames-and-extrusion.md) and [Z travel tests](maintenance-tasks.md#z-travel-and-scanning) as the baseline.

## 2. Print lifecycle and DMS controls

### Pause and stop transitions

`Printer::pause` owns the pause phase, the configured park offsets, the saved return position, the direct extrusion to recover and the standby deadline. Panel input, queued G-code and DMS submit requests. Timer1 reads one byte to decide whether the print queue can advance. A pause still lets the direct moves and live Z compensation run. The pause handler sets the active phase before entering waits that can service the same handler again. [Pause implementation](../Repetier/PrintPause.cpp)

| Event | Transition |
| --- | --- |
| First panel Pause | Freeze the queue, save the direct XYZ position, and retract E. The second Pause lifts Z and parks XY. |
| Queued M25, M3070 or DMS pause | Accept the request in place or with a park according to its source. A queued pause command returns when a pause freezes the queue. |
| Automatic or requested standby | Reduce the hotend target and motor current while keeping the saved return position. The third panel Pause requests immediate standby after parking. |
| Play | Restore heat first if needed. After a reheat the next Play starts the return. Complete XY before Z, then recover E. A clipped return leaves the print paused for another attempt. |
| Stop | Cancel the pause and end any active pause jog. The stop sequence drains input and waits for motion to finish, turns off the heaters and releases the command block after the output delay. |

`Printer::printStop` keeps the abort phase and its timestamp separately. The pause and stop timers use elapsed time across clock rollover. [Stop implementation](../Repetier/PrintStop.cpp) · [Lifecycle replay](../tests/pause-sense-offset.py)

The AVR builds use 4 fewer bytes of global SRAM and 76–136 fewer bytes of flash than `48e0c99`. The Timer1 pause check uses the same instructions. Instruction traces for queued X and Z, direct Z and idle compensation match the previous cycle counts on all three model builds. Physical pulse timing still needs verification.

### Job boundaries and DMS

- **Initialize serial jobs at Z homing.** The standard start G-code runs `G28` before enabling compensation or priming. The queue-depth heuristic serves the printing status. [USB print start](first-layer-control/README.md#usb-print-start)
- **Drain work at normal SD file end.** The current EOF path enters the abort sequence, which discards queued motion. Track completion after the file's buffered G-code and movement have run. [Deferred SD task](maintenance-tasks.md#sd-print-completion)
- **Make pressure control follow phase.** Give start-line braking, first-layer SenseOffset, elastic Digit-Z, and emergency pressure actions clear inputs and effects. Define when each takes effect in the queue and on the machine. Keep `M3912`'s force-sensitive XY and E brake independently observable from M220/M221, and test its cleanup on completion, skip and cancellation. Check first-layer recognition with and without a Z move after priming, including 0.35 and 0.352 mm. [Priming and DMS behavior](first-layer-control/m3912-state-boundary.md)
- **Make Digit-Z settle consistently.** Retain the filter's rounding remainder and replay increasing, decreasing and steady loads. Measure deflection and the delay from load change through DMS reading to applied Z at the printer. [Digit-Z finding and measurement plan](z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)

## 3. Calibration and persistence

- **Trace measurement to activation.** Follow full-bed scans and point Z-offset scans through contact detection, the working RAM matrix, the active sector and reload. Keep the current operator workflows as compatibility cases, including explicit matrix saves and transferring roughly 5 µm from SenseOffset to the saved offset with each Mod Menu DOWN/UP pair. [Scans and matrix storage](z-offset/scans-and-persistence.md) · [Mod Menu transfer](z-offset/history-and-workflows.md)
- **Make `M3902 Z0` recoverable.** Define how the internal EEPROM offset and external EEPROM matrix are committed and recovered as a pair. Verify the active sector and test a restart after each write and recovery step, including a second interruption during recovery. Reload either the complete previous pair or the complete new pair, with the expected effective correction in either case. Verify that the ordinary load path still reads existing matrix sectors and stored offsets. [Current transfer sequence](z-offset/scans-and-persistence.md#critical-transfer-operation-m3902-z0)
- **Preserve compatible settings on firmware upgrade.** Internal EEPROM is built in. It uses the signature `44` at address 0, an integrity byte at 104 and layout version 8 at 105. Boot loads the settings when the signature and checksum match. Keep addresses stable, use free slots for new fields and assign valid defaults when loading images that predate them. [Boot handling](../Repetier/Eeprom.cpp) · [Addresses](../Repetier/Eeprom.h)

## 4. Motion smoothness and performance

- **Measure the causes of uneven motion.** Use the same G-code and settings before and after each change. Compare long moves with dense short segments. Measure planner throughput and queue depth, reasons for gaps between segments, ISR duration, step interval and pulse widths. Include active Z-CMP and cases with pressure advance or DMS speed limiting. Count XYZ/E pulses alongside timing to catch motion changes hidden by a faster run.
- **Review junctions and waits.** Check the junction-speed calculation and short-segment slowdown together. Trace Pressure Advance through lookahead and step execution. Replay the fork's inter-segment Z-CMP wait change against queued motion, blocked corrections and paused prints that still need live compensation. [Motion review tasks](maintenance-tasks.md#motion-smoothness-and-performance)
- **Evaluate step packing in that baseline.** Compare using the computed `minInterval` with the 64-tick pressure-advance allowance and changing the 512-tick floor in separate experiments. Judge each by measured motion continuity, pulse timing and AVR interrupt headroom, with matching step totals. [Step-packing diff](https://github.com/afriess/RFx000x-Firmware/commit/0e11752)

## 5. Firmware structure and hardware variants

- **Split responsibilities where the workflows cross.** As each block settles its state and callers, bring G-code and panel actions through the same machine operations where they request the same behavior. Review which parts of `RF.cpp` belong to motion policy, print lifecycle, calibration or peripherals. Review periodic and nested waits for explicit completion conditions and a single owner of each transition. Expose small interfaces that eliminate direct cross-module writes.
- **Review actual hardware differences.** Compare board pins, driver and timer behavior, endstop topology and calibration data. Keep model differences at the hardware or configuration boundary where they express real capabilities. Use the same exercise for heater, watchdog and peripheral hooks when those boundaries are touched. [Board selection](../Repetier/Configuration.h#L45) · [Pin mappings](../Repetier/pins.h)
- **Make refactoring verifiable.** Move in small slices with a stated state owner, call path and expected behavior. Keep structural changes and intentional behavior changes in separate commits where practical. Bring representative host replays into the repository as the shared interfaces take shape. Run the relevant replays and AVR builds for RF1000, RF2000 and RF2000v2, track flash and SRAM, and record physical verification separately for motion, sensors and persistence. [Existing printer scenarios](first-layer-control/test-scenarios.md)

### Feature retirement

The RFx000 port started from Repetier firmware designed for many printers and boards. It inherited configuration markers and alternate paths from that general-purpose code. Some of those options lack an implementation or a connected device in the current RFx000 builds. Check the RFx000 callers, factory profiles and retrofit wiring before treating an inherited option as a supported variant.

Inventory optional features and combinations before reorganizing their callers. For each, record actual usage, hardware and G-code dependencies, and the cost of keeping the alternate path. Decide whether to retain it, fold overlapping behavior into one path, or retire it. Prioritize features that add branches across several subsystems. Review the resulting number of supported configurations and the branches removed from shared code.

**Milling mode is retired.** RF1000, RF2000 and RF2000v2 now have one print path through homing, motion, calibration and the menus. Legacy milling commands follow the ordinary unknown-command path. Internal EEPROM settings and external heat-bed matrix sectors use their existing addresses and format versions. The previously stored operating-mode byte and the former workpiece matrix slots are reserved. [Last revision containing milling code](https://github.com/Nibbels/Repetier-Firmware/tree/7e2e08b7f6da7677007dfddd06f8fc5adeb7d709) · [Internal EEPROM](../Repetier/Eeprom.h) · [Matrix storage](../Repetier/Configuration.h)

**Ten optional print and development features are retired.** Ditto printing (`M280`), position memory (`M401`/`M402`), standalone parking (`M3080`/`M3103`), the three secondary axis-driver switches, service reminders, caliper input (`M3999`), viscosity testing (`M3939`), and the INT3 test counter (`M3989`) were disabled in all three stock configurations. Pause parking and return use their own coordinates, configurable with `M3105`. The sample end code calls `M3079` to present the printed part. Each tool has its own extrusion controls on dual-extruder models. Print statistics report cumulative time and filament usage. EEPROM addresses 169–176 and 1934–1936 are reserved for the retired settings. [Last revision containing the optional code](https://github.com/Nibbels/Repetier-Firmware/tree/39211536066802d07e716bb0f885471a268d057a)

**The RF1000 uses a software Z maximum.** The physical switch provides Z-min and the configured height limits travel toward Z-max. The factory setting is 195 mm. An existing EEPROM value takes precedence, including 200 mm from an earlier firmware. `M205` reports the height at address 153 and `M119` reports the software Z maximum as `soft_z_max:`. The retired mode byte at address 1016 is reserved. RF2000 and RF2000v2 use their separate Z-min and Z-max inputs with factory heights of 200 mm and 185 mm. [RF1000 configuration](../Repetier/RF1000.h) · [Z limit checks](../Repetier/Printer.h)

**The print builds share one compensation and DMS configuration.** Bed scanning and Z compensation, Digit Flow, Digit-Z, SenseOffset, DMS pause and stop, host DMS reporting and M3912 priming are compiled for all three models. Runtime settings and EEPROM addresses still control their behavior. The firmware accepts linear moves with `G0` and `G1`. Slicer output must use linear segments for curves since `G2` and `G3` were retired with the arc planner. Servo output and `M340` are retired.

**Lighting follows the board pins.** All models offer the switched X19 output. RF2000 and RF2000v2 also have an independent RGB output with white as the factory mode. The RGB settings use the RGB pin mapping and keep their existing EEPROM values. [Board pin mappings](../Repetier/pins.h)

**Acoustic signals are retired.** The menu, audible temperature alerts and tone commands M120 and M300 were removed. The board-type check halts setup on a mismatch and reports the problem on the serial port. Digit Homing and the movement lock after reset or motor disable are compiled for all three models. The Digit Homing menu and its stored preference control when the DMS reference is applied. EEPROM address 1000 is reserved.

**Core controls are built into all three models.** Microstep adjustment, board-type verification, LCD access and button actions (`M3029`), and part-cooling fan control compile for each model. Board selection enables the 230 V output, 24 V FET outputs and case fan on RF2000 and RF2000v2. The case fan uses FET3 on X45 in both stock configurations. Guided nozzle alignment (`M3309`) and the DRV8711 status command (`M3987`) are retired. On dual-extruder machines, the bed scan now runs through to the matrix save without a button press for alignment. The PLA and ABS scans still measure the thermal change in nozzle height.

**Temperature correction follows the model's calibration.** RF1000 uses zero offsets. RF2000 and RF2000v2 use −2, −4, −6, −8, −10 and −10 °C at sensor readings of 60, 80, 100, 120, 140 and 160 °C, with interpolation. The heater regulates against the corrected reading. The common conversion preserves the previous calibration of the bed, hotend and spare sensors. [Temperature conversion](../Repetier/Extruder.cpp#L729)

**Homing diagnostics and WobbleFix are built in.** The homing repeatability command `M3028` works on all models. WobbleFix uses the existing menu and EEPROM settings, with the factory amplitudes at zero. The integer checks skip the trigonometry and XY offset arithmetic at zero. The planner applies configured corrections while rounding XY step endpoints. [Planner](../Repetier/Printer.cpp#L285) · [Wobble replay](maintenance-tasks.md#wobble-correction)

**G-code checksum validation stays in the protocol.** ASCII checksums, binary Fletcher-16 and the serial check on the switch back to ASCII use their existing parser paths. The former compile-time switch set an unused bit. The three model builds now have zero `FEATURE_*` switches. [G-code parser](../Repetier/gcode.cpp)

**The shared build uses the stock motion and display settings.** Pressure Advance is built in and its per-tool factory coefficient is zero. The current junction and short-segment calculations and the 25 ms motor enable wait are built in. The panel keeps the selected info page and returns to it after 120 seconds of menu inactivity. SD builds include the file size in `M20`. Bed-scan state tracing and periodic RAM probes are retired. `M3200 P1` still samples RAM on demand. The room-temperature threshold is 33 °C.

**The host uses the firmware's UART0 driver.** Normal acknowledgements include the line number. `M111` controls the debug level and command echo. The boot level is 6 (info and errors), the idle serial signal is `wait`, and long operations send `busy:` messages at a 2 s interval. The transmit ring reserves 64 bytes, with 63 available for outgoing data. The parsed G-code queue holds two commands at 88 bytes each. `MOVE_CACHE_SIZE` holds 17 planned moves with SD support or 24 without it, at 119 bytes per move in the AVR build. [Serial driver](../Repetier/HAL.h#L159) · [Command queue](../Repetier/gcode.cpp) · [Move cache](../Repetier/motion.cpp#L74)

**The experimental TipDown hotend is retired.** Tool changes apply the X and Y offsets. The former `M3919` adjustment and menu are removed, and offsets 95–98 in each internal EEPROM extruder block are reserved. The printer-wide manual Z offset continues to use its own EEPROM address. [Tool selection](../Repetier/Extruder.cpp#L323) · [EEPROM addresses](../Repetier/Eeprom.h)

**M109 sets or waits for the hotend temperature.** Retraction during heat-up belongs in slicer output or start G-code. Offsets 50–53 in each internal EEPROM extruder block are reserved for the old retract settings. The hotend profiles define the mandatory setpoint limit and normal heater cutoff, 275 °C for V1, V2 and V3. The built-in sensor IDs retain their values. Retired custom sensor IDs 5–7, 50–52 and 97–99 are treated as sensor faults if present in older EEPROM data. [Hotend profiles](../Repetier/Configuration.h) · [Temperature control](../Repetier/Extruder.cpp) · [EEPROM addresses](../Repetier/Eeprom.h)

**All RF builds include the heated bed, physical min endstops, character LCD and DRV8711 driver.** The RF2000 boards also wire a physical Z-max endstop. All boards use the stock STEP pulse timing and heater and cooler PWM. The English LCD menu and encoder settings are compiled in. Builds with `SDSUPPORT` use hardware SPI, table-driven CRC, single-block I/O and one shared cache for FAT16 and FAT32. [Board pins](../Repetier/pins.h) · [SdFat implementation](../Repetier/SdFat.cpp)

**RF2000 and RF2000v2 share a board map and configuration.** Their factory travel limits, Z switch parameters, extruder calibration, hotends, bed PID, scan motion and part fan settings remain model-specific. Each board supports one or two extruders. The shared firmware targets ATmega2560 and homes requested axes toward their min switches in XYZ order. Named board connectors and their retrofit roles are listed in the [board reference](board-connectors.md).

**Temperature inputs and hotend defaults have one implementation.** The ADC samples T0, T1 when installed, then the heated bed. The RF2000 family adds X35 as the last channel. The hotend profiles define temperature limits and supply PID defaults for startup and factory restore. Each tool can select its profile and sensor independently. Tool changes update the LCD directly, and the host's `M117` remains available. [Hotend profiles and overrides](../Repetier/Configuration.h) · [Temperature control](../Repetier/Extruder.cpp)

**Scan and pause defaults are shared where the models agree.** The bed scan uses one set of grid and probe defaults, with X margins selected by tool count and the coarse Z retract selected by model. The Z-offset scan uses one fine probe per attempt and retains its existing retry path. Boot and factory restore both compute the XYZ pause offsets with the corresponding axes' steps per mm. The active tool supplies the E retract and `M3105` still sets pause offsets. [Scan defaults and pause setup](../Repetier/RF.cpp)

**The fitted peripherals use their existing wiring.** The heated bed and part fan are present in every build. SD builds use the fixed chip-select wiring, with an active-low card detect on RF2000 and RF2000v2. The RF2000 family's mains output starts off. X45 retains its EEPROM setting and starts under case-fan control. [Board reference](board-connectors.md)
