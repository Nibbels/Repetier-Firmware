# Firmware maintenance handover

**Development roadmap.** Build firmware for the RFx000 boards with the established Repetier serial printing interface. Give each service ownership of its mutable state and use small, verifiable commits. Evaluate step packing in the motion smoothness and performance block. [Development blocks](docs/development-roadmap.md) · [Maintenance tasks](docs/maintenance-tasks.md)

**Verified coordinate semantics.** `Printer::layers` owns the pending and confirmed model layers and both fade boundaries in nominal planned Z steps. The foreground compares them with the nominal position executing in `ZMotion`. The M3912 patch restores the incoming logical E coordinate. [Coordinate frames and extrusion timeline](docs/first-layer-control/coordinate-frames-and-extrusion.md) · [M3912 patch](https://github.com/Nibbels/Repetier-Firmware/commit/5f848c9a5cd0347c6d45fd9d3f13afd7344cb775)

**Extrusion planning.** `Printer::extrusionPlanner` owns the last queued logical E and fractional base step. Use `Printer::setEPositionMM()` to synchronize the current and queued E reference while retaining the fractional step. [Extrusion planner](Repetier/ExtrusionPlanner.h)

**Geometric extrusion compensation.** `Printer::extrusionCompensation` owns the ECMP ratio and fractional extra step. The foreground prepares the ratio and publishes it alongside the Z correction target under interrupt protection. The timer accounts for the E direction and normalizes the remainder between pulses when advance is off. [ECMP](Repetier/ExtrusionCompensation.h)

**Move targets.** `Printer::moveTarget` owns the retained machine-space millimetres, their fractional carry and the work offset. The planner asks for these values before rounding to steps. [Coordinate targets](Repetier/MoveTarget.h)

**M3912 uses an isolated pressure-braked priming phase.** `Printer::startLine` owns the phase, generates its moves and restores the temporary state after draining the queue. High DMS force progressively constrains XY speed and reduces E flow, like the audible change when a chainsaw enters wood. Manual start code enables SenseOffset with `M3909` after priming. [State ledger and test plan](docs/first-layer-control/m3912-state-boundary.md)

**Start with [docs/first-layer-control/README.md](docs/first-layer-control/README.md) before editing first-layer, Z compensation, DMS or priming behavior. For the full Z-offset sum, boot/print lifecycle, matrix EEPROM and thermal/scanning risks, read [docs/z-offset/README.md](docs/z-offset/README.md).** The linked documentation explains original intent, actual execution paths, risks and test scenarios.

This is shared AVR/ATmega2560 Repetier firmware for **RF1000, RF2000 and RF2000v2**. Motion planning, SenseOffset and bed correction are common code. Do not apply model-only changes without a demonstrated model-specific cause. Stepper timing and SRAM/flash limits matter; avoid expensive arithmetic in ISRs.

**Motion performance.** Optimize sustainable print speed and worst-case service gaps together. Use the [timing plan](docs/motion-timing.md) for STEP/DIR ownership, Advance handoffs, UART throughput and the baseline required to evaluate packing. Use the [planner notes](docs/motion-timing.md#lookahead-short-moves-and-extrusion-transitions) when touching short-segment speed, retract or print–travel joins. Preserve their behavior during structural refactors and review the speed policy with its own replays.

**Heater safety and watchdog.** Scan and motion waits must keep periodic services running. Read the [heater safety and watchdog requirements](docs/heater-safety.md) before changing heater control, PWM, scheduler waits or scan state dispatch.

**Motion planning.** `MotionPlanner` prepares the step profile and owns the junction calculations. The planner updates queue slots in place and releases them with complete profiles. For a direct move, `Printer::offsetRelativeStepsCoordinates()` reserves the slot under interrupt protection, then the foreground plans and publishes the move. Replay early jog release and queue wrap with `python3 tests/pause-sense-offset.py`. [Planner and executor boundary](docs/motion-timing.md#planner-and-executor-boundary)

**Step execution.** `StepExecutor` owns both step clocks, cached primary-axis counts, pulse packing, direction and the feedrate snapshot for the panel. Timer1 selects the move and the executor runs its prepared profile. Keep hardware operations in the executor and check the AVR pulse traces when changing a hot path. [Executor](Repetier/StepExecutor.cpp)

**Board connectors and retrofits.** Preserve named physical connector definitions and the existing controls for available switched outputs. A port can be unused in the factory build and used for equipment such as an E3D V6 or Volcano heatsink fan or a water-cooling pump. The RF2000 family shares its pin map and configuration with model-specific defaults. Check the [board notes and X44 example](docs/board-connectors.md) before changing pin assignments or retiring an output path.

**Design essentials.** DMS measures strain-gauge force. Excess force may indicate inadequate nozzle clearance, a clog or material, temperature or speed differences. SenseOffset (`M3909`) adds bounded positive clearance on the first layer. Bed-profile compensation follows the scanned bed, then fades its XY variation across later layers with geometric E-material compensation (ECMP). `M3912` primes at nominal Z=0.35 mm and suspends layer learning and SenseOffset for the duration of the line. The learned `fadeStartSteps()` boundary controls both the start of profile fade and eligibility for SenseOffset.

**Z motion.** `Printer::zMotion` owns the execution counters and travel rules. Use its operations for queued, direct, scan and compensation steps. A clipped nominal step or a step balanced by compensation can advance the path without an emitted pulse. Run `python3 tests/z-motion.py` when editing these paths. [Execution model](docs/development-roadmap.md#z-step-execution)

**XY travel limits.** `Printer::xyMotion` owns executed nominal steps, motor position, direct displacement and skipped queue steps. Queued motion pays back skipped steps on the way back, including when the switch has opened. A direct retreat preserves the queued position. [XY limits and replay](docs/maintenance-tasks.md#xy-travel-limits)

**Bed compensation.** `BedCompensation` calculates the bed-profile correction and ECMP ratio from executing nominal Z, planned layer bounds and a matrix reader. `recalculateZCompensation()` adds the effective offset and active Digit-Z, then publishes the Z target and ECMP ratio together under interrupt protection. [Calculation and test cases](docs/first-layer-control/z-compensation-and-ecmp.md)

**Digit compensation.** `Printer::digitCompensation` owns the DMS filter, the Digit-Z switch, the Digit Flow profile and its live factors. Keep sampling and regulation in the foreground. Timer1 reads the atomically published `intervalScale()` for queued XY moves. M220 and M221 still control the user speed and flow overrides. Run `python3 tests/pause-sense-offset.py` for the DMS sample and M3912 replays. [DMS controls](docs/development-roadmap.md#job-boundaries-and-dms)

**Strain gauge.** `Printer::strainGauge` owns ADC access and the Digit Homing reference. Keep the I²C driver independent of motion. `sampleStrainGauge()` supplies the corrected sample to the filter and to its caller. [Driver](Repetier/StrainGauge.cpp)

**SenseOffset.** `Printer::senseOffsetControl` owns the active limits, AutoStart and the pressure learning window. `ZOffset` owns the applied clearance. The foreground proposes an increment from one snapshot of the limits and checks that snapshot and the print phase when applying it. The step queue can activate stored limits during the calculation. [SenseOffset settings and replay](docs/first-layer-control/sense-offset.md)

**DMS pressure protection.** `Printer::pressureProtection` owns the limits and sampling windows. The foreground requests pause and stop actions and publishes the E inhibit with the output owner. [Pressure checks](docs/development-roadmap.md#job-boundaries-and-dms)

**Extrusion output.** `Printer::extrusionOutput` owns the Advance timer mode, pending E balance, pressure lead and motor inhibits. Settle the old motion before changing tool or Advance settings. Check the shared inhibit before admitting new E or lead work. A pressure trip or abort clears the old balance under interrupt protection. Run `python3 tests/pause-sense-offset.py` for the handoff cases. [STEP ownership](docs/motion-timing.md#step-and-direction-ownership)

**Pause and stop.** `Printer::pause` owns the pause phase and return position. Manual E uses direct steps and preserves the print's logical E and fractional carry. Once a manual jog moves filament, Play keeps that prepared position. `Printer::printJob` owns print and filament-service admission, the menu's busy state and timed abort cleanup. Run `python3 tests/pause-sense-offset.py` when changing these transitions. [Pause and stop transitions](docs/development-roadmap.md#pause-and-stop-transitions)

**Idle and shutdown.** `Printer::idleStatus` owns the pending Ready transition shared by the foreground and Timer1. `Printer::inactivity` owns the last activity time and the two persisted `M84` and `M85` thresholds. Use elapsed clock arithmetic through rollover and take the inactivity sample after preceding foreground services. [Lifecycle tests](tests/pause-sense-offset.py) · [Timeout replay](tests/machine-inactivity.cpp)

**Print input.** `UsbSerial` owns USART0 and the byte rings. `GCodeInput` owns the receiver state and two parsed command slots. All three models buffer 24 planned moves. Run `python3 tests/serial-protocol.py` when changing transport, admission or framing. [Serial compatibility](docs/development-roadmap.md#serial-printing-compatibility) · [SD support removal](docs/maintenance-tasks.md#sd-support-removal)

**Non-negotiable operator workflow:** In Mod Menu page 2 the Z/bed DOWN→UP button pair intentionally transfers 5 µm of live SenseOffset into the EEPROM-backed manual part of `Printer::zOffset` with approximately unchanged final clearance (see [live takeover procedure](docs/z-offset/history-and-workflows.md) and the original January 2017 changelog). Do not “fix” the asymmetric button behavior or reset dynamic offsets while a manual transfer is underway.

**Print start.** The standard USB start sequence runs `G28` before enabling bed compensation. Z homing restores the fade bounds and clears layer tracking, ECMP and live SenseOffset with its sample history. `M3079` runs the output move and disables the motors and bed compensation at the end of a print. [USB print start](docs/first-layer-control/README.md#usb-print-start) · [Test scenarios](docs/first-layer-control/test-scenarios.md)

**Git workflow:** `ai-firmware-review` is the isolated working branch; keep `community_development` unchanged until specifically instructed. The DRV8711 BLANK invalid read command was removed without changing actual default electrical timings in [commit 8f00ed8](https://github.com/Nibbels/Repetier-Firmware/commit/8f00ed8df6a0e825dee368c1233ffc4934349faa).

For future chats: read this file and the mechanism docs, verify against code and changelog, distinguish verified behavior from hypotheses, then propose isolated changes and tests rather than guessing.
